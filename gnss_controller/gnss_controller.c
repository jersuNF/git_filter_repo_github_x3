#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>
#include "gnss_controller.h"
#include "gnss_controller_events.h"
#include "error_event.h"
#include "gnss.h"
#include "kernel.h"
#define STACK_SIZE 1024
#define PRIORITY 7

#define GNSS_1SEC 1000

K_SEM_DEFINE(new_data_sem, 0, 1);

#define MODULE gnss_controller
LOG_MODULE_REGISTER(MODULE, CONFIG_GNSS_CONTROLLER_LOG_LEVEL);

#define MS_IN_49_DAYS 4233600000
static _Noreturn void publish_gnss_data(void *ctx);
static int gnss_data_update_cb(const gnss_t *);
static void gnss_timed_out(void);
static int gnss_set_mode(gnss_mode_t mode, bool wakeup);

static gnss_t gnss_data_buffer;
static gnss_mode_t current_mode = GNSSMODE_NOMODE;
static uint16_t current_rate_ms = UINT16_MAX;

const struct device *gnss_dev = NULL;
static uint8_t gnss_reset_count;
static uint8_t gnss_timeout_count;
static uint8_t gnss_failed_init_count;

K_THREAD_STACK_DEFINE(pub_gnss_stack, STACK_SIZE);
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

static int gnss_controller_setup(void)
{
	int ret = gnss_wakeup(gnss_dev);
	if (ret != 0) {
		LOG_ERR("gnss_wakeup failed %d", ret);
		return ret;
	}
	ret = gnss_setup(gnss_dev, false);
	if (ret != 0) {
		LOG_ERR("gnss_setup failed %d", ret);
		return ret;
	}
	/* See https://content.u-blox.com/sites/default/files/documents/MIA-M10Q_IntegrationManual_UBX-21028173.pdf
     * Section 2.1, receiver configuration. MIA M10 resets clear the configuration, so a reset must be followed
     *  By a new configuration
     * The configuration stored in BBR is also cleared by a RESET_N signal or a UBX-CFG-RST message
     * with reset mode set to a hardware reset (resetMode 0x00 and 0x04), but otherwise will be used as
     * long as the backup battery supply remains.
     * CAUTION The configuration interface has changed from earlier u-blox positioning receivers.
     * Users must adopt the configuration interface described in this document.
     */
	return ret;
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

int gnss_controller_init(void)
{
	enum States {
		ST_GNSS_FW_DRIVER_INIT,
		ST_GNSS_FW_CB_INIT,
		ST_GNSS_HW_INIT,
		ST_GNSS_RUNNING
	};
	enum States current_state = ST_GNSS_FW_DRIVER_INIT;
	int ret;
	gnss_reset_count = 0;
	gnss_timeout_count = 0;
	current_mode = GNSSMODE_NOMODE;
	gnss_failed_init_count = 0;
	while (gnss_failed_init_count < CONFIG_GNSS_INIT_MAX_COUNT) {
		LOG_INF("current state: %i, N= %i, ret= %i", current_state, gnss_failed_init_count,
			ret);
		switch (current_state) {
		case ST_GNSS_FW_DRIVER_INIT: {
			gnss_dev = DEVICE_DT_GET(DT_ALIAS(gnss));
			if (gnss_dev == NULL) {
				char *msg = "Couldn't get instance of the GNSS device!";
				nf_app_error(ERR_GNSS_CONTROLLER, -1, msg, sizeof(*msg));
				gnss_failed_init_count++;
			} else {
				current_state++;
			}
		} break;
		case ST_GNSS_FW_CB_INIT: {
			ret = gnss_set_data_cb(gnss_dev, gnss_data_update_cb);
			if (ret != 0) {
				char *msg = "Failed to register data CB!";
				nf_app_error(ERR_GNSS_CONTROLLER, ret, msg, sizeof(*msg));
				current_state--;
				gnss_failed_init_count++;
			} else {
				current_state++;
			}
		} break;
		case ST_GNSS_HW_INIT: {
			ret = gnss_controller_setup();
			if (ret != 0) {
				char *msg = "Failed setup of GNSS!";
				nf_app_error(ERR_GNSS_CONTROLLER, ret, msg, sizeof(*msg));
				current_state--;
				gnss_failed_init_count++;
			} else {
				current_state++;
			}
		} break;
		case ST_GNSS_RUNNING: {
#if defined(CONFIG_TEST)
			pub_gnss_thread_id =
#endif
				k_thread_create(&pub_gnss_thread, pub_gnss_stack,
						K_KERNEL_STACK_SIZEOF(pub_gnss_stack),
						(k_thread_entry_t)publish_gnss_data, (void *)NULL,
						NULL, NULL, PRIORITY, 0, K_NO_WAIT);
			return 0;
		} break;
		default:
			break;
		}
	}
	LOG_ERR("GNSS init failed %i, after %i tries.", ret, gnss_failed_init_count);
	//TODO: add hard reset
	return -1;
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
			int ret = gnss_set_mode(ev->mode, true);
			if (ret != 0) {
				LOG_ERR("Failed to set mode %d", ret);
				char *msg = "Failed to set GNSS receiver mode";
				nf_app_error(ERR_GNSS_CONTROLLER, ret, msg, sizeof(*msg));
				return false;
			}
			current_mode = ev->mode;
		}
		return false;
	}
	return false;
}

EVENT_LISTENER(MODULE, gnss_controller_event_handler);
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
