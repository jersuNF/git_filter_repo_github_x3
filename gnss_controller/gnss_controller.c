
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <date_time.h>
#include "gnss_controller.h"
#include "gnss_controller_events.h"
#include "error_event.h"
#include "gnss.h"
#include "kernel.h"
#include "UBX.h"
#include "messaging_module_events.h"
#include "storage.h"
#include "timeutil.h"
#include "nclogs.h"

#define UPDATE_ANO_PRIORITY 12

#include <pm/pm.h>

#define PRIORITY 7

#define SECONDS_IN_12_HOURS (12 * 3600)

K_SEM_DEFINE(new_data_sem, 0, 1);
K_SEM_DEFINE(install_ano_sem, 0, 1);
K_SEM_DEFINE(ano_thread_cmd_issued_sem, 0, 1);
#define MODULE gnss_controller

#define MS_IN_49_DAYS 4233600000
static _Noreturn void publish_gnss_data(void *ctx);
static int gnss_data_update_cb(const gnss_t *);
static void gnss_timed_out(void);
static int gnss_set_mode(gnss_mode_t mode, bool wakeup);
static void gnss_thread_fn(void);
static _Noreturn void gnss_ano_install_thread_fn(void);

static gnss_t gnss_data_buffer;
static gnss_mode_t current_mode = GNSSMODE_NOMODE;
static uint16_t current_rate_ms = UINT16_MAX;
const struct device *gnss_dev = NULL;
static uint8_t gnss_reset_count = 0;
static uint8_t gnss_timeout_count = 0;

K_THREAD_STACK_DEFINE(pub_gnss_stack, CONFIG_GNSS_PUBLISH_STACK_SIZE);

K_THREAD_DEFINE(send_to_gnss, CONFIG_GNSS_STACK_SIZE, gnss_thread_fn, NULL, NULL, NULL,
		K_PRIO_COOP(CONFIG_GNSS_THREAD_PRIORITY), 0, 0);

enum gnss_action_e {
	GNSS_ACTION_NONE = 0,
	GNSS_ACTION_SET_MODE,
	GNSS_ACTION_INTERNAL_ANO_UPLOAD_STARTING,
	GNSS_ACTION_INTERNAL_ANO_UPLOAD_FINISHED,
};

typedef struct gnss_msgq_t {
	enum gnss_action_e action;
	void *arg;
} gnss_msgq_t;

K_THREAD_STACK_DEFINE(update_ano_stack, CONFIG_GNSS_ANO_THREAD_STACK_SIZE);
struct k_thread update_ano_thread;

K_MSGQ_DEFINE(gnss_msgq, sizeof(gnss_msgq_t), 1, 4);

#ifdef CONFIG_GNSS_DATA_BATCH
/* Work queue item for triggering a batch read from the GNSS receiver */
static struct k_work_delayable retrieve_batch_gnss_work;
#endif

/**
 * @brief Set GNSS mode with retries
 * @param[in] mode the mode to
 * @return 0 on success, otherwise negative error code
 */

static int set_mode_with_retry(gnss_mode_t mode)
{
	int rc = 0;
	int retries = CONFIG_GNSS_CMD_RETRIES;
	do {
		rc = gnss_set_mode(mode, true);
	} while ((retries-- > 0) && (rc != 0));
	return rc;
}

static void gnss_thread_fn(void)
{
	bool is_in_ano_install = false;
	bool set_to_backup_when_ano_finished = false;

	while (true) {
		int rc = 0;
		gnss_msgq_t msg;
		k_msgq_get(&gnss_msgq, &msg, K_FOREVER);

		switch (msg.action) {
		case GNSS_ACTION_SET_MODE: {
			gnss_mode_t mode = (gnss_mode_t)(msg.arg);
			if (is_in_ano_install && mode == GNSSMODE_INACTIVE) {
				/* ANO is in progress. If the mode is backup don't set it until ANO has finished */
				NCLOG_DBG(GNSS_CONTROLLER, TRice0(iD( 4603),"dbg: ANO in progress. Setting mode to backup when ANO is finished \n"));
				set_to_backup_when_ano_finished = true;
				rc = 0;
			} else {
				set_to_backup_when_ano_finished = false;
				rc = set_mode_with_retry(mode);
			}
			if (rc == 0) {
				current_mode = mode;
			} else {
				NCLOG_ERR(GNSS_CONTROLLER, TRice( iD( 5785),"err: Failed to set mode %d with %d retries \n", rc, CONFIG_GNSS_CMD_RETRIES));
				char *msg = "Failed to set GNSS receiver mode";
				nf_app_error(ERR_GNSS_CONTROLLER, rc, msg, sizeof(*msg));
			}

			/* Send out an event (to the amc_handler)to answer with current mode */
			struct gnss_mode_changed_event *ev = new_gnss_mode_changed_event();
			ev->mode = current_mode;
			EVENT_SUBMIT(ev);
			break;
		}
		case GNSS_ACTION_INTERNAL_ANO_UPLOAD_STARTING: {
			/* If we are in backup, we must wake the receiver until ANO finished */
			is_in_ano_install = true;
			if (current_mode == GNSSMODE_INACTIVE) {
				set_to_backup_when_ano_finished = true;
				int rc = gnss_wakeup(gnss_dev);
				if (rc != 0) {
					NCLOG_WRN(GNSS_CONTROLLER, TRice(iD(4417), "Cannot wake up the receiver %d \n",rc));
				}
			}
			/* Notify the ANO thread that it can continue */
			k_sem_give(&ano_thread_cmd_issued_sem);
			break;
		}
		case GNSS_ACTION_INTERNAL_ANO_UPLOAD_FINISHED: {
			is_in_ano_install = false;
			if (set_to_backup_when_ano_finished) {
				int rc = gnss_set_backup_mode(gnss_dev);
				if (rc != 0) {
					NCLOG_WRN(GNSS_CONTROLLER,TRice(iD( 5946),"Cannot set internal backup mode %d \n", rc));
				}
			}
			/* Notify the ANO thread that it can continue */
			k_sem_give(&ano_thread_cmd_issued_sem);
			break;
		}

		default:
			NCLOG_ERR(GNSS_CONTROLLER, TRice( iD( 3470),"Unrecognized action %d \n", msg.action));
		}
	}
}

#ifdef CONFIG_GNSS_DATA_BATCH
static void gnss_retrieve_batch_work_fn(struct k_work *work)
{
	gnss_retrieve_batch(gnss_dev);
	k_work_reschedule(&retrieve_batch_gnss_work, K_MSEC(current_rate_ms));
}
#endif

struct k_thread pub_gnss_thread;
static bool initialized = false;

#if defined(CONFIG_TEST)
k_tid_t pub_gnss_thread_id;
k_tid_t ano_gnss_thread_id;
#endif

/** @brief Sends a timeout event from the GNSS controller */
static void gnss_controller_send_timeout_event(void)
{
	gnss_t gnss_no_fix;
	memset(&gnss_no_fix, 0, sizeof(gnss_t));

	struct gnss_data *new_data = new_gnss_data();
	new_data->gnss_data = gnss_no_fix;
	new_data->timed_out = true;

	EVENT_SUBMIT(new_data);
}

/** @brief Reset and initialize GNSS  */
static int gnss_controller_reset_and_setup_gnss(uint16_t mask)
{
	int ret = gnss_wakeup(gnss_dev);
	if (ret != 0) {
		NCLOG_ERR(GNSS_CONTROLLER, TRice(iD(6612), "gnss_wakeup failed %d \n", ret));
		return ret;
	}

	ret = gnss_reset(gnss_dev, mask, GNSS_RESET_MODE_HW_IMMEDIATELY);
	if (ret != 0) {
		NCLOG_ERR(GNSS_CONTROLLER, TRice(iD(1294), "gnss_reset failed %d \n", ret));
		return ret;
	}

	/* @todo config in BBR is cleared for HW resets, we must re-configure for now */
	/* See https://content.u-blox.com/sites/default/files/documents/MIA-M10Q_IntegrationManual_UBX-21028173.pdf 2.1 */
	ret = gnss_setup(gnss_dev, false);
	if (ret != 0) {
		NCLOG_ERR(GNSS_CONTROLLER, TRice(iD(5302), "gnss_setup failed %d \n", ret));
		return ret;
	}

	ret = gnss_set_mode(current_mode, false);
	if (ret != 0) {
		NCLOG_ERR(GNSS_CONTROLLER, TRice(iD(3142), "gnss_set_mode %d \n", ret));
		return ret;
	}

	return 0;
}

int gnss_controller_setup(void)
{
	int ret = -ENOSYS;
	struct ublox_mon_ver mia_m10_versions;

	ret = gnss_set_data_cb(gnss_dev, gnss_data_update_cb);
	if (ret != 0) {
		NCLOG_WRN(GNSS_CONTROLLER, TRice(iD(5900), "Failed to register data CB! %d \n", ret));
		return ret;
	}

	ret = gnss_setup(gnss_dev, false);
	if (ret != 0) {
		NCLOG_WRN(GNSS_CONTROLLER, TRice(iD(3789), "gnss_setup failed %d \n", ret));
		return ret;
	}

	ret = gnss_set_mode(current_mode, false);
	if (ret != 0) {
		NCLOG_ERR(GNSS_CONTROLLER, TRice(iD(7118), "gnss_set_mode %d \n", ret));
		return ret;
	}

	// Default values to please ztest
	mia_m10_versions.swVersion[0] = 0;
	mia_m10_versions.hwVersion[0] = 0;
	// Query the mia m10 for SW version and HW version and print them.
	ret = gnss_version_get(gnss_dev, &mia_m10_versions);
	if (ret != 0) {
		NCLOG_WRN(GNSS_CONTROLLER,TRice(iD(4558), "wrn: Failed to get version of mia m10! %d \n", ret));
		return ret;
	}
	// NCLOG_INF(GNSS_CONTROLLER, trice(iD(1177), "The GNSS SW Version:%s",log_strdup(mia_m10_versions.swVersion)));
	// NCLOG_INF(GNSS_CONTROLLER, trice(iD(4026), "The GNSS Hardware Version:%s",log_strdup(mia_m10_versions.hwVersion)));

	return ret;
}

int gnss_controller_init(void)
{
	int ret = -ENOSYS;

	current_mode = GNSSMODE_NOMODE;
	gnss_timeout_count = 0;
	gnss_reset_count = 0;

	gnss_dev = DEVICE_DT_GET(DT_ALIAS(gnss));
	if (!gnss_dev) {
		return -ENODEV;
	}

#ifdef CONFIG_GNSS_DATA_BATCH
	/* Init and start GNSS data batch retrieval work item */
	k_work_init_delayable(&retrieve_batch_gnss_work, gnss_retrieve_batch_work_fn);
#endif

	while (gnss_reset_count++ < CONFIG_GNSS_MAX_COUNT_RESETN_RESTART) {
		ret = gnss_resetn_pin(gnss_dev);
		if (ret != 0) {
			char *msg = "gnss_resetn_pin toggle failed";
			nf_app_error(ERR_GNSS_CONTROLLER, ret, msg, sizeof(*msg));
			return ret;
		}

		/* Give GNSS receiver some time to initialize after reset */
		k_sleep(K_MSEC(100));

		ret = gnss_controller_setup();
		if (ret == 0) {
			break;
		}
		NCLOG_INF(GNSS_CONTROLLER, TRice(iD(2616), "RESTART [ iteration=%d, ret=%d ]\n", gnss_reset_count, ret));
	}

#if defined(CONFIG_TEST)
	pub_gnss_thread_id =
#else
	k_tid_t thread =
#endif

		k_thread_create(&pub_gnss_thread, pub_gnss_stack,
				K_KERNEL_STACK_SIZEOF(pub_gnss_stack),
				(k_thread_entry_t)publish_gnss_data, (void *)NULL, NULL, NULL,
				PRIORITY, 0, K_NO_WAIT);
#if !defined(CONFIG_TEST)
	k_thread_name_set(thread, "gnss_ctrler");
#endif

#if defined(CONFIG_TEST)
	ano_gnss_thread_id =
#endif
		k_thread_create(&update_ano_thread, update_ano_stack,
				K_KERNEL_STACK_SIZEOF(update_ano_stack),
				(k_thread_entry_t)gnss_ano_install_thread_fn, NULL, NULL, NULL,
				UPDATE_ANO_PRIORITY, 0, K_NO_WAIT);
	return 0;
}

static _Noreturn void publish_gnss_data(void *ctx)
{
	int ret;
	int timeout_ms;
	while (true) {
		if (current_rate_ms == UINT16_MAX ||
		    current_rate_ms < CONFIG_GNSS_MINIMUM_ALLOWED_GNSS_RATE) {
			timeout_ms = CONFIG_GNSS_DEFAULT_TIMEOUT_RATE_MS;
		} else {
			/* Allow some slack on GNSS solution */
			timeout_ms = current_rate_ms + CONFIG_GNSS_TIMEOUT_SLACK_MS;
		}
		if ((ret = k_sem_take(&new_data_sem, K_MSEC(timeout_ms))) == 0) {
			gnss_reset_count = 0;
			gnss_timeout_count = 0;
			struct gnss_data *new_data = new_gnss_data();
			new_data->gnss_data = gnss_data_buffer;
			new_data->timed_out = false;
			NCLOG_DBG(GNSS_CONTROLLER,TRice( iD( 3224),"GNSS: lon: %d, psmState: 0x%02x, hAcc: %d, numSv: %d, mode: %d, fix: %d valid 0x%02x\n",gnss_data_buffer.latest.lon, (gnss_data_buffer.latest.pvt_flags >> 2),gnss_data_buffer.latest.h_acc_dm, gnss_data_buffer.latest.num_sv, gnss_data_buffer.latest.mode,gnss_data_buffer.fix_ok,gnss_data_buffer.latest.pvt_valid));

			EVENT_SUBMIT(new_data);
			initialized = true;
		} else {
			if (initialized && current_mode != GNSSMODE_INACTIVE) {
				gnss_timed_out();
			}
		}
		if (gnss_data_buffer.latest.msss >= MS_IN_49_DAYS && gnss_reset_count == 0) {
			gnss_controller_reset_and_setup_gnss(GNSS_RESET_MASK_COLD);
		}
	}
}

static int gnss_data_update_cb(const gnss_t *data)
{
	memcpy(&gnss_data_buffer, data, sizeof(gnss_t));
	k_sem_give(&new_data_sem);
	return 0;
}

static int gnss_set_mode(gnss_mode_t mode, bool wakeup)
{
	int ret;
	uint16_t rate_ms;
	if (wakeup) {
		ret = gnss_wakeup(gnss_dev);
		if (ret != 0) {
			NCLOG_ERR(GNSS_CONTROLLER, TRice(iD(6081), "gnss_wakeup failed %d \n", ret));
			return ret;
		}
	}

	if (mode == GNSSMODE_INACTIVE) {
		ret = gnss_set_backup_mode(gnss_dev);
		if (ret != 0) {
			NCLOG_ERR(GNSS_CONTROLLER, TRice(iD(7764), "err: failed to set GNSS in backup %d \n", ret));
			return ret;
		}
		current_rate_ms = UINT16_MAX;

		return 0;
	}

	/* TODO: Add rate as an argument to gnss_set_mode */
	switch (mode) {
	case GNSSMODE_PSM:
		rate_ms = GNSSRATE_5000_MS;
		break;

	case GNSSMODE_CAUTION:
		rate_ms = GNSSRATE_1000_MS;
		break;

	case GNSSMODE_MAX:
		rate_ms = GNSSRATE_250_MS;
		break;

	default:
		rate_ms = GNSSRATE_1000_MS;
		break;
	}
	NCLOG_INF(GNSS_CONTROLLER, TRice(iD(4929), "Setting GNSS mode to %u with rate: %u \n", mode, rate_ms));
	ret = gnss_set_power_mode(gnss_dev, mode, rate_ms);
	if (ret != 0) {
		NCLOG_ERR(GNSS_CONTROLLER, TRice(iD( 6433), "failed to set GNSS to mode %u %d \n", mode, ret));
		return ret;
	}

#ifdef CONFIG_GNSS_DATA_BATCH
	/* Retrive data batch */
	k_work_reschedule(&retrieve_batch_gnss_work, K_MSEC(current_rate_ms));
#endif
	/*
         * Update the current_rate_ms used to check GNSS timeout AFTER mode has been set
         * Otherwise, we might risk GNSS timeout in the receiver thread
         */
	current_rate_ms = rate_ms;
	return 0;
}

void gnss_force_backup()
{
	gnss_set_mode(GNSSMODE_INACTIVE, false);
}

void gnss_force_psmct()
{
	gnss_set_mode(GNSSMODE_PSM, false);
}

void gnss_force_wakeup()
{
	gnss_set_mode(GNSSMODE_CAUTION, true);
}

static bool gnss_controller_event_handler(const struct event_header *eh)
{
	if (is_gnss_set_mode_event(eh)) {
		struct gnss_set_mode_event *ev = cast_gnss_set_mode_event(eh);
		NCLOG_DBG(GNSS_CONTROLLER, TRice(iD(5295), "MODE = %d old = %d \n", ev->mode, current_mode));
		if (ev->mode != current_mode) {
			gnss_msgq_t msg;
			msg.arg = (void *)(ev->mode);
			msg.action = GNSS_ACTION_SET_MODE;
			k_msgq_put(&gnss_msgq, &msg, K_NO_WAIT);
		}
		return false;
	}
	if (is_ano_ready_event(eh)) {
		k_sem_give(&install_ano_sem);
		return false;
	}
	return false;
}

EVENT_LISTENER(MODULE, gnss_controller_event_handler);
EVENT_SUBSCRIBE(MODULE, gnss_data_rate);
EVENT_SUBSCRIBE(MODULE, gnss_switch_off);
EVENT_SUBSCRIBE(MODULE, gnss_switch_on);
EVENT_SUBSCRIBE(MODULE, gnss_set_mode);
EVENT_SUBSCRIBE(MODULE, ano_ready_event);
EVENT_SUBSCRIBE(MODULE, gnss_set_mode_event);

/**
 * @brief Handles GNSS timeouts when no messages has been received.
 *        Will reset GNSS with various modes depending on reset count.
 *
 * @return None
 *
 */
static void gnss_timed_out(void)
{
	/* Always send an event */
	gnss_timeout_count++;
	gnss_controller_send_timeout_event();

	/* be much more careful to send reset events than timeouts, as this might stress the GNSS receiver */
	if (gnss_timeout_count > CONFIG_GNSS_TIMEOUTS_BEFORE_RESET) {
		gnss_timeout_count = 0;
		NCLOG_DBG(GNSS_CONTROLLER, TRice(iD(5063), "resets GNSS %d \n", gnss_reset_count));
		if (gnss_reset_count < 1) {
			gnss_reset_count++;
			gnss_controller_reset_and_setup_gnss(GNSS_RESET_MASK_HOT);
		} else if (gnss_reset_count < 2) {
			gnss_reset_count++;
			gnss_controller_reset_and_setup_gnss(GNSS_RESET_MASK_WARM);
		} else if (gnss_reset_count >= 2) {
			gnss_reset_count++;
			gnss_controller_reset_and_setup_gnss(GNSS_RESET_MASK_COLD);
		}
	}
}

/* TODO; the whole storage design makes update very cumbersome as the callback function
 * must decide if storage should continue reading records or not
 * We should re-design the whole storage API to work without relying on callback
 * returns */

static ano_rec_t _ano_rec;
static uint32_t _ano_unix_time_sec;

static inline int read_ano_callback(uint8_t *data, size_t len)
{
	int ret = 0;
	if (len != sizeof(ano_rec_t)) {
		NCLOG_ERR(GNSS_CONTROLLER, TRice( iD( 7945),"Error reading ano entry from flash was %d expected %d \n", len, sizeof(UBX_MGA_ANO_RAW_t)));
		return -EINVAL;
	}
	memcpy(&_ano_rec, data, sizeof(_ano_rec));
	uint16_t last_good_ano_id = UINT16_MAX;
	uint32_t ano_time_md = ano_date_to_unixtime_midday(_ano_rec.raw_ano.mga_ano.year,
							   _ano_rec.raw_ano.mga_ano.month,
							   _ano_rec.raw_ano.mga_ano.day);
	/* check that the ano data is from the current day or in the future */
	int err = stg_config_u16_read(STG_U16_LAST_GOOD_ANO_ID, &last_good_ano_id);
	if (err) {
		NCLOG_WRN(GNSS_CONTROLLER, TRice(iD(3989), "Failed to read STG_U16_LAST_GOOD_ANO_ID %d \n", err));
	}
	if (ano_time_md + SECONDS_IN_12_HOURS >= _ano_unix_time_sec &&
	    (last_good_ano_id == 0 || last_good_ano_id == 0xFFFF ||
	     last_good_ano_id == _ano_rec.ano_id)) {
		ret = gnss_upload_assist_data(gnss_dev, &_ano_rec.raw_ano);
		if (ret == 0) {
			NCLOG_DBG(GNSS_CONTROLLER, TRice( iD( 2010),"Installed ano ID=%d,off=%d,time=%d \n", _ano_rec.ano_id, _ano_rec.sequence_id, ano_time_md));
		} else {
			NCLOG_ERR(GNSS_CONTROLLER, TRice(iD(6817), "Failed to install ano %d \n", ret));
		}
	} else {
		NCLOG_DBG(GNSS_CONTROLLER, TRice( iD( 6288),"DROPS ano ID=%d,off=%d,time=%d \n", _ano_rec.ano_id, _ano_rec.sequence_id, ano_time_md));
	}
	return ret;
}

/**
 * @brief install new ano data to GNSS receiver.
 */
_Noreturn void gnss_ano_install_thread_fn(void)
{
	bool all_ano_installed = false;
	bool start_from_flash = true;
	bool gnss_receiver_got_time = false;
	int ret;

	/* Wait for valid date time before we start installing */
	/* TODO, we could use the date_time_event_handler to set a semaphore instead */
	while (!date_time_is_valid()) {
		k_sleep(K_MSEC(200));
	}

	while (true) {
		all_ano_installed = false;

		/*
		 * Notify the GNSS commander thread that we are about to uoload ANO and require
		 * that the GNSS receiver is woken up
		 */
		gnss_msgq_t msg = { .action = GNSS_ACTION_INTERNAL_ANO_UPLOAD_STARTING };
		k_msgq_put(&gnss_msgq, &msg, K_NO_WAIT);
		/* Wait for the receiver to be woken up */
		k_sem_take(&ano_thread_cmd_issued_sem, K_FOREVER);

		while (!all_ano_installed) {
			int64_t unix_time_ms = 0;
			ret = date_time_now(&unix_time_ms);
			if (ret != 0) {
				NCLOG_ERR(GNSS_CONTROLLER, TRice(iD(7282), "Cannot get date_time %d \n", ret));
				break;
			}

			/* The callback needs the unix time to decide relevant ANO records */
			_ano_unix_time_sec = unix_time_ms / 1000;

			if (!gnss_receiver_got_time) {
				struct tm tmbuf;
				time_t t = _ano_unix_time_sec;
				if (!gmtime_r(&t, &tmbuf)) {
					NCLOG_ERR(GNSS_CONTROLLER, TRice0(iD(5918), "gmtime_r failed \n"));
					break;
				}
				ret = gnss_ini_mga_time_utc(gnss_dev, tmbuf.tm_year + 1900,
							    tmbuf.tm_mon + 1, tmbuf.tm_mday,
							    tmbuf.tm_hour, tmbuf.tm_min,
							    tmbuf.tm_sec, 10);
				if (ret != 0) {
					NCLOG_ERR(GNSS_CONTROLLER, TRice(iD(2303), "err: Cannot set GNSS time %d \n", ret));
					break;
				}
				gnss_receiver_got_time = true;
			}

			/* Note, most of the processing is done in the read_ano_callback: */
			int err = stg_read_ano_data(read_ano_callback, start_from_flash, 1);
			if (err == 0) {
				start_from_flash = false;
			} else if (err == -ENODATA) {
				NCLOG_DBG(GNSS_CONTROLLER, trice0(iD(6415), "dbg: Reached end of ano partition \n"));
				all_ano_installed = true;
			} else {
				NCLOG_ERR(GNSS_CONTROLLER, trice0(iD(1454), "err: Failed to read ano entry from flash! \n"));
				/* This could happen if the GNSS receiver is in backup mode */
				/* break the loop and wait for semaphore */
				break;
			}
		}
		/*
		 * Notify the GNSS commander thread that we are finished uploading ANO
		 * and that the receiver might be put to sleep if it should have been
         	 */
		msg.action = GNSS_ACTION_INTERNAL_ANO_UPLOAD_FINISHED;
		k_msgq_put(&gnss_msgq, &msg, K_NO_WAIT);
		/* Wait for the command to finish */
		k_sem_take(&ano_thread_cmd_issued_sem, K_FOREVER);

		/*
		* Wait forever for new data coming in, or, if we encountered an error,
	 	* try again after 5 minutes.
		*/
		ret = k_sem_take(&install_ano_sem, all_ano_installed ? K_FOREVER : K_MINUTES(5));
		if (ret != 0) {
			NCLOG_WRN(GNSS_CONTROLLER, trice0(iD(5222), "wrn: Retrying ANO install \n"));
		}
	}
}