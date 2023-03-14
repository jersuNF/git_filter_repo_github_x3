#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>
#include "gnss_controller.h"
#include "gnss_controller_events.h"
#include "error_event.h"
#include "gnss.h"
#include "kernel.h"
#include "nf_eeprom.h"
#include "UBX.h"
#include "messaging_module_events.h"
#include "storage.h"
#include "unixTime.h"

#define NEW_GNSS_DATA_STACK_SIZE 1024
#define NEW_GNSS_DATA_PRIORITY 7

#define UPDATE_ANO_STACK_SIZE 1024
#define UPDATE_ANO_PRIORITY 12

#define STACK_SIZE 1024
#define PRIORITY 7
#define GNSS_1SEC 1000

#define UNIX_TIMESTAMP_1_JAN_2000 1451606400

K_SEM_DEFINE(new_data_sem, 0, 1);

K_SEM_DEFINE(ano_update, 0, 1);

#define MODULE gnss_controller
LOG_MODULE_REGISTER(MODULE, CONFIG_GNSS_CONTROLLER_LOG_LEVEL);

#define MS_IN_49_DAYS 4233600000
static _Noreturn void publish_gnss_data(void *ctx);
static int gnss_data_update_cb(const gnss_t *);
static void gnss_timed_out(void);
static int gnss_set_mode(gnss_mode_t mode, bool wakeup);
static void gnss_thread_fn(void);
static _Noreturn void install_fresh_ano(void);

static gnss_t gnss_data_buffer;
static gnss_mode_t current_mode = GNSSMODE_NOMODE;
static uint16_t current_rate_ms = UINT16_MAX;
const struct device *gnss_dev = NULL;
static uint8_t gnss_reset_count = 0;
static uint8_t gnss_timeout_count = 0;

K_THREAD_STACK_DEFINE(pub_gnss_stack, STACK_SIZE);

K_THREAD_DEFINE(send_to_gnss, CONFIG_GNSS_STACK_SIZE, gnss_thread_fn, NULL, NULL, NULL,
		K_PRIO_COOP(CONFIG_GNSS_THREAD_PRIORITY), 0, 0);

enum gnss_action_e { GNSS_ACTION_NUL = 0, GNSS_ACTION_SET_MODE };

typedef struct gnss_msgq_t {
	enum gnss_action_e action;
	void *arg;
} gnss_msgq_t;

K_THREAD_STACK_DEFINE(update_ano_stack, UPDATE_ANO_STACK_SIZE);
struct k_thread update_ano_thread;

K_MSGQ_DEFINE(gnss_msgq, sizeof(gnss_msgq_t), 1, 4);

static void gnss_thread_fn(void)
{
	while (true) {
		gnss_msgq_t msg;
		k_msgq_get(&gnss_msgq, &msg, K_FOREVER);

		switch (msg.action) {
		case GNSS_ACTION_SET_MODE: {
			LOG_DBG("setting mode");
			int rc = 0;
			int retries = CONFIG_GNSS_CMD_RETRIES;
			gnss_mode_t mode = (gnss_mode_t)(msg.arg);
			do {
				rc = gnss_set_mode(mode, true);
			} while ((retries-- > 0) && (rc != 0));

			if (rc == 0) {
				current_mode = mode;
			} else {
				LOG_ERR("Failed to set mode %d with %d retries", rc,
					CONFIG_GNSS_CMD_RETRIES);
				char *msg = "Failed to set GNSS receiver mode";
				nf_app_error(ERR_GNSS_CONTROLLER, rc, msg, sizeof(*msg));
			}
			/* Send out an event (to the amc_handler)to answer with current mode */
			struct gnss_mode_changed_event *ev = new_gnss_mode_changed_event();
			ev->mode = current_mode;
			EVENT_SUBMIT(ev);
			break;
		}
		default:
			LOG_ERR("Unrecognized action %d", msg.action);
		}
	}
}

struct k_thread pub_gnss_thread;
static bool initialized = false;

#if defined(CONFIG_TEST)
k_tid_t pub_gnss_thread_id;
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
		LOG_ERR("gnss_wakeup failed %d", ret);
		return ret;
	}

	ret = gnss_reset(gnss_dev, mask, GNSS_RESET_MODE_HW_IMMEDIATELY);
	if (ret != 0) {
		LOG_ERR("gnss_reset failed %d", ret);
		return ret;
	}

	/* @todo config in BBR is cleared for HW resets, we must re-configure for now */
	/* See https://content.u-blox.com/sites/default/files/documents/MIA-M10Q_IntegrationManual_UBX-21028173.pdf 2.1 */
	ret = gnss_setup(gnss_dev, false);
	if (ret != 0) {
		LOG_ERR("gnss_setup failed %d", ret);
		return ret;
	}
	ret = gnss_set_mode(current_mode, false);
	if (ret != 0) {
		LOG_ERR("gnss_set_mode %d", ret);
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
		LOG_WRN("Failed to register data CB! %d", ret);
		return ret;
	}

	ret = gnss_wakeup(gnss_dev);
	if (ret != 0) {
		LOG_WRN("gnss_wakeup failed %d", ret);
		return ret;
	}

	ret = gnss_setup(gnss_dev, false);
	if (ret != 0) {
		LOG_WRN("gnss_setup failed %d", ret);
		return ret;
	}
	// Default values to please ztest
	mia_m10_versions.swVersion[0] = 0;
	mia_m10_versions.hwVersion[0] = 0;
	// Query the mia m10 for SW version and HW version and print them.
	ret = gnss_version_get(gnss_dev, &mia_m10_versions);
	if (ret != 0) {
		LOG_WRN("Failed to get version of mia m10! %d", ret);
		return ret;
	}

	LOG_INF("The GNSS SW Version:%s", log_strdup(mia_m10_versions.swVersion));
	LOG_INF("The GNSS Hardware Version:%s", log_strdup(mia_m10_versions.hwVersion));
	return ret;
}

int gnss_controller_init_retry_softcold_setup(void)
{
	int ret = -ENOSYS;
	uint8_t gnss_init_count = 0;

	while (gnss_init_count < CONFIG_GNSS_MAX_COUNT_INIT) {
		LOG_INF(" Trying INIT [count=%d, ret=%d]", gnss_init_count, ret);
		ret = gnss_controller_setup();
		if (ret == 0) {
			return ret;
		}
		gnss_init_count++;
	}

	return ret;
}

int gnss_controller_init_retry_softcold(void)
{
	int ret = -ENOSYS;
	gnss_reset_count = 0;

	ret = gnss_controller_init_retry_softcold_setup();
	if (ret == 0) {
		return ret;
	}

	while (gnss_reset_count < CONFIG_GNSS_MAX_COUNT_SOFT_RESTART) {
		ret = gnss_reset(gnss_dev, GNSS_RESET_MASK_COLD, GNSS_RESET_MODE_HW_IMMEDIATELY);

		gnss_reset_count++;
		LOG_INF("RESTART [ iteration=%d, ret=%d ]", gnss_reset_count, ret);

		ret = gnss_controller_init_retry_softcold_setup();
		if (ret == 0) {
			return ret;
		}
	}

	return ret;
}

int gnss_controller_init_retry(void)
{
	int ret = -ENOSYS;

	ret = gnss_controller_init_retry_softcold();
	if (ret == 0) {
		return ret;
	}
	// If SOFT-COLD resets fail, try to toggle the resetn pin for a hard reset.
	ret = gnss_resetn_pin(gnss_dev);
	if (ret != 0) {
		char *msg = "gnss_resetn_pin toggle failed";
		nf_app_error(ERR_GNSS_CONTROLLER, ret, msg, sizeof(*msg));
		return ret;
	}

	ret = gnss_controller_init_retry_softcold();

	return ret;
}

int gnss_controller_init(void)
{
	current_mode = GNSSMODE_NOMODE;
	gnss_timeout_count = 0;
	int ret;

	gnss_dev = DEVICE_DT_GET(DT_ALIAS(gnss));
	if (!gnss_dev) {
		return -ENODEV;
	}

	ret = gnss_controller_init_retry();
	if (ret != 0) {
		char *msg = "gnss init failed after using the resetn pin";
		nf_app_error(ERR_GNSS_CONTROLLER, ret, msg, sizeof(*msg));
		return ret;
	}

#if defined(CONFIG_TEST)
	pub_gnss_thread_id =
#endif
		k_thread_create(&pub_gnss_thread, pub_gnss_stack,
				K_KERNEL_STACK_SIZEOF(pub_gnss_stack),
				(k_thread_entry_t)publish_gnss_data, (void *)NULL, NULL, NULL,
				PRIORITY, 0, K_NO_WAIT);
	return 0;
	k_thread_create(&update_ano_thread, update_ano_stack,
			K_KERNEL_STACK_SIZEOF(update_ano_stack),
			(k_thread_entry_t)install_fresh_ano, NULL, NULL, NULL, UPDATE_ANO_PRIORITY,
			0, K_NO_WAIT);
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
			LOG_DBG("  GNSS data: %d, %d, %d, %d, %d", gnss_data_buffer.latest.lon,
				gnss_data_buffer.latest.lat, gnss_data_buffer.latest.pvt_flags,
				gnss_data_buffer.latest.h_acc_dm, gnss_data_buffer.latest.num_sv);
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
	if (wakeup) {
		ret = gnss_wakeup(gnss_dev);
		if (ret != 0) {
			LOG_ERR("gnss_wakeup failed %d", ret);
			return ret;
		}
	}

	if (mode == GNSSMODE_INACTIVE) {
		ret = gnss_set_backup_mode(gnss_dev);
		if (ret != 0) {
			LOG_ERR("failed to set GNSS in backup %d", ret);
			return ret;
		}
		current_rate_ms = UINT16_MAX;
	} else if (mode == GNSSMODE_PSM || mode == GNSSMODE_CAUTION || mode == GNSSMODE_MAX) {
		ret = gnss_set_power_mode(gnss_dev, mode);
		if (ret != 0) {
			LOG_ERR("failed to set GNSS to mode %u %d", mode, ret);
			return ret;
		}
		ret = gnss_get_rate(gnss_dev, &current_rate_ms);
		if (ret != 0) {
			LOG_ERR("failed to get rate %d", ret);
			return ret;
		}
	}

	return 0;
}

static bool gnss_controller_event_handler(const struct event_header *eh)
{
	if (is_gnss_set_mode_event(eh)) {
		struct gnss_set_mode_event *ev = cast_gnss_set_mode_event(eh);
		LOG_DBG("MODE = %d old = %d ", ev->mode, current_mode);
		if (ev->mode != current_mode) {
			LOG_DBG("setting mode");
			gnss_msgq_t msg;
			msg.arg = (void *)(ev->mode);
			msg.action = GNSS_ACTION_SET_MODE;
			k_msgq_put(&gnss_msgq, &msg, K_NO_WAIT);
		}
		return false;
	}
	if (is_ano_ready(eh)) {
		k_sem_give(&ano_update);
		return false;
	}
	return false;
}

EVENT_LISTENER(MODULE, gnss_controller_event_handler);
EVENT_SUBSCRIBE(MODULE, gnss_data_rate);
EVENT_SUBSCRIBE(MODULE, gnss_switch_off);
EVENT_SUBSCRIBE(MODULE, gnss_switch_on);
EVENT_SUBSCRIBE(MODULE, gnss_set_mode);
EVENT_SUBSCRIBE(MODULE, ano_ready);
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
		LOG_DBG("resets GNSS %d", gnss_reset_count);
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

static uint32_t ano_date_to_unixtime_midday(uint8_t year, uint8_t month, uint8_t day)
{
	nf_time_t nf_time;
	nf_time.day = day;
	nf_time.month = month;
	nf_time.year = year + 2000;
	nf_time.hour = 12; //assumed per ANO specs
	nf_time.minute = nf_time.second = 0;
	return time2unix(&nf_time);
}

int install_ano_msg(GPS_UBX_MGA_ANO_t *new_ano_message)
{
	return gnss_upload_assist_data(gnss_dev, (uint8_t *)new_ano_message,
				       sizeof(GPS_UBX_MGA_ANO_t));
}

GPS_UBX_MGA_ANO_t ano_msg;
static inline int read_ano_callback(uint8_t *data, size_t len)
{
	uint32_t ano_msg_age;
	ano_msg_age = ano_date_to_unixtime_midday(ano_msg.year, ano_msg.month, ano_msg.day);
	LOG_INF("ano msg age=%d, svID =%d", ano_msg_age, ano_msg.svId);
	if (len != sizeof(GPS_UBX_MGA_ANO_t)) {
		LOG_ERR("Error reading ano entry from flash, size not "
			"correct!");
		return -EINVAL;
	}
	/* Memcpy the flash contents. */
	memset(&ano_msg, 0, len);
	memcpy(&ano_msg, data, len);
	return 0;
}

/**
 * @brief install new ano data to GNSS receiver.
 * Starts as soon as new data is available on the flash,
 *  iterates on the ano messages in flash and updates the last ano record
 *  (day) on eeprom on success of installation of all messages for that day.
 * In case of failure, sleep for 1 minute before retrying.
 */
_Noreturn void install_fresh_ano(void)
{
	static uint32_t last_installed_ano_time, serial_id;
	static bool all_ano_installed = false, ano_downloaded = false;
	uint32_t ano_msg_age = 0;
	int ret;
	/* todo, get rid off this */
	static uint32_t current_time;

	while (true) {
		current_time = gnss_data_buffer.lastfix.unix_timestamp;
		if (current_time > UNIX_TIMESTAMP_1_JAN_2000) {
			if (!all_ano_installed) {
				//			k_sem_take(&ano_update, K_FOREVER);
				int err = stg_read_ano_data(read_ano_callback, false, 1);
				if (err == 0) {
					/*check timestamp and install if fresh.*/
					ano_msg_age = ano_date_to_unixtime_midday(
						ano_msg.year, ano_msg.month, ano_msg.day);
					LOG_WRN("time: %d, ano_msg_time: %d", current_time,
						ano_msg_age);
					if (ano_msg_age > current_time &&
					    ano_msg_age >= last_installed_ano_time) {
						ret = install_ano_msg(&ano_msg);
						if (ret == 0) {
							last_installed_ano_time = ano_msg_age;
							LOG_INF("Installed ano "
								"msg!");
						} else {
							LOG_INF("Failed to "
								"install "
								"ano msg! ,%d",
								ret);
						}
						//TODO: handle failure to install ano_msg
					}
					//					else {
					//						LOG_INF("Finished installing "
					//							" all ano from flash!");
					//						all_ano_installed = true;
					//					}
				} else if (err == -ENODATA) {
					LOG_INF("Reached end of ano "
						"partition!");
					all_ano_installed = true;
					last_installed_ano_time = ano_msg_age;
				} else {
					LOG_ERR("Failed to read ano entry from flash!");
				}
			}
			if (serial_id == 0) {
				int err = eep_read_serial(&serial_id);
				if (err != 0) {
					LOG_ERR("Failed to read serial id from eeprom!");
				}
				LOG_INF("Serial number = %i", serial_id);
			}
			/* adding  a random offset to avoid storming the
			server with update ano requests from all collars
			at the same time. */
			uint16_t offset = (serial_id & 0x0FF) * 56;
			if ((last_installed_ano_time + offset < current_time) &&
			    all_ano_installed && !ano_downloaded) {
				/* Time to download new ano data from the server*/
				struct start_ano_download *ev = new_start_ano_download();
				LOG_INF("Submitting req_ano!");
				EVENT_SUBMIT(ev);
				int err = stg_clear_partition(STG_PARTITION_ANO);
				if (err != 0) {
					LOG_ERR("Failed to clear ano "
						"partition!");
				}
				if (k_sem_take(&ano_update, K_MINUTES(5)) == 0) {
					all_ano_installed = false;
					ano_downloaded = true;
				}
				/* wait until ano download is complete or
				 * timeout expires. */
			}
		}
		//		k_yield();
		k_sleep(K_MSEC(200));
	}
}