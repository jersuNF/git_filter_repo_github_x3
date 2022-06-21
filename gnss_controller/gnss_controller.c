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
#define GNSS_5SEC (GNSS_1SEC * 5)
#define GNSS_10SEC (GNSS_1SEC * 10)
#define GNSS_20SEC (GNSS_1SEC * 20)
#define GNSS_DATA_TIMEOUT (GNSS_1SEC * 25)

K_SEM_DEFINE(new_data_sem, 0, 1);
K_SEM_DEFINE(cached_fix_sem, 1, 1);

#define MODULE gnss_controller
LOG_MODULE_REGISTER(MODULE, CONFIG_GNSS_CONTROLLER_LOG_LEVEL);

#define MIN_GNSS_RATE CONFIG_MINIMUM_ALLOWED_GNSS_RATE

static _Noreturn void publish_gnss_data(void *ctx);
static int gnss_data_update_cb(const gnss_t *);
static void gnss_timed_out(void);
static void gnss_clear_reset_count(void);
static int gnss_set_mode(gnss_mode_t mode);

static uint16_t current_rate;

static gnss_t gnss_data_buffer;
gnss_mode_t current_mode = GNSSMODE_NOMODE;

gnss_t cached_gnss_data;

const struct device *gnss_dev = NULL;
uint8_t gnss_reset_count;

K_THREAD_STACK_DEFINE(pub_gnss_stack, STACK_SIZE);
struct k_thread pub_gnss_thread;
bool pub_gnss_started = false;

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
	int ret = gnss_setup(gnss_dev, false);
	if (ret != 0) {
		char *msg = "Failed to set up GNSS receiver!";
		nf_app_error(ERR_GNSS_CONTROLLER, ret, msg, sizeof(*msg));
		return ret;
	}
	ret = gnss_set_rate(gnss_dev, 250);
	if (ret != 0) {
		char *msg = "Failed to set GNSS receiver data rate!";
		nf_app_error(ERR_GNSS_CONTROLLER, ret, msg, sizeof(*msg));
		return ret;
	}
	ret = gnss_get_rate(gnss_dev, &current_rate);
	if (ret != 0) {
		char *msg = "Failed to get GNSS receiver data rate!";
		nf_app_error(ERR_GNSS_CONTROLLER, ret, msg, sizeof(*msg));
		return ret;
	}
	ret = 0; // gnss_get_mode(gnss_dev, &current_mode);
	if (ret != 0) {
		char *msg = "Failed to get GNSS receiver mode!";
		nf_app_error(ERR_GNSS_CONTROLLER, ret, msg, sizeof(*msg));
		return ret;
	}

	return ret;
}

/** @brief Reset and initialize GNSS, send notification event. */
static int gnss_controller_reset_gnss(uint16_t mask) 
{
	gnss_controller_send_timeout_event();

	gnss_reset(gnss_dev, mask,
			GNSS_RESET_MODE_HW_IMMEDIATELY);

	return gnss_controller_setup();
}

int gnss_controller_init(void)
{
	printk("Initializing gnss controller!\n");
	gnss_dev = DEVICE_DT_GET(DT_ALIAS(gnss));
	if (gnss_dev == NULL) {
		char *msg = "Couldn't get instance of the GNSS device!";
		printk("%s", msg);
		nf_app_error(ERR_GNSS_CONTROLLER, -1, msg, sizeof(*msg));
		return -1;
	}
	int ret = gnss_set_data_cb(gnss_dev, gnss_data_update_cb);
	if (ret != 0) {
		char *msg = "Failed to register data CB!";
		nf_app_error(ERR_GNSS_CONTROLLER, ret, msg, sizeof(*msg));
		return ret;
	}

	ret = gnss_controller_setup();
	if (ret != 0) {
		char *msg = "Failed setup of GNSS!";
		nf_app_error(ERR_GNSS_CONTROLLER, ret, msg, sizeof(*msg));
		return ret;
	}

	if (!pub_gnss_started) {
		k_thread_create(&pub_gnss_thread, pub_gnss_stack,
				K_KERNEL_STACK_SIZEOF(pub_gnss_stack),
				(k_thread_entry_t)publish_gnss_data,
				(void *)NULL, NULL, NULL, PRIORITY, 0,
				K_NO_WAIT);
		pub_gnss_started = true;
	}

	/* power consumption crude test - begin
	ret = gnss_set_rate(gnss_dev, MIN_GNSS_RATE);
	if(ret != 0){
		LOG_WRN("Switched to 4Hz!\n");
	}
	while (true){
		LOG_WRN("Attempting GNSS stop!\n");
		ret = gnss_reset(gnss_dev, GNSS_RESET_MASK_COLD,
				 GNSS_RESET_MODE_GNSS_STOP);
		if (ret != 0){
			LOG_WRN("Finished GNSS stop!\n");
		}

		k_sleep(K_SECONDS(30));
		LOG_WRN("Attempting GNSS start!\n");
		ret = gnss_reset(gnss_dev, GNSS_RESET_MASK_COLD,
				 GNSS_RESET_MODE_GNSS_START);
		if (ret != 0){
			LOG_WRN("Finished GNSS start!\n");
		}
	}
power consumption crude test - end */
	return 0;
}

static _Noreturn void publish_gnss_data(void *ctx)
{
	while (true) {
		if (k_sem_take(&new_data_sem, K_SECONDS(5)) == 0) {
			gnss_clear_reset_count();

			struct gnss_data *new_data = new_gnss_data();
			new_data->gnss_data = gnss_data_buffer;
			new_data->timed_out = false;
			LOG_INF("  GNSS data: %d, %d, %d, %d, %d", gnss_data_buffer.latest.lon,
				gnss_data_buffer.latest.lat, gnss_data_buffer.latest.pvt_flags, gnss_data_buffer.latest.h_acc_dm,
				gnss_data_buffer.latest.num_sv);
			EVENT_SUBMIT(new_data);
			if (k_sem_take(&cached_fix_sem, K_MSEC(100)) == 0) {
				cached_gnss_data = gnss_data_buffer;
				k_sem_give(&cached_fix_sem);
			} else { /* TODO: take action if needed. */
				LOG_WRN("Failed to update cached GNSS fix!\n");
			}
		} else {
			gnss_timed_out();
		}
	}
}

static int gnss_data_update_cb(const gnss_t *data)
{
	memcpy(&gnss_data_buffer, data, sizeof(gnss_t));
	k_sem_give(&new_data_sem);
	return 0;
}

static int gnss_set_mode(gnss_mode_t mode)
{
	/** @todo Add mode changes here. Should be in the gnss driver???? */
	switch (mode) {
	case GNSSMODE_INACTIVE:
		/*GPS_SetPowerMode_Backup();*/
		break;
	case GNSSMODE_PSM:
		/*GPS_SetPowerMode_PSM(PSM_DEFAULT_PACC_M);*/
		break;
	case GNSSMODE_CAUTION:
		/*GPS_SetPowerMode_CAUTION();*/
		break;
	case GNSSMODE_MAX:
		/*GPS_SetPowerMode_MAX();*/
		break;
	default:
		break;
	}

	/*GPS_UpdatePowerMode();*/
	return 0;
}

static bool gnss_controller_event_handler(const struct event_header *eh)
{
	if (is_gnss_rate(eh)) {
		struct gnss_rate *ev = cast_gnss_rate(eh);
		if (ev->rate >= MIN_GNSS_RATE && current_rate != ev->rate) {
			int ret = gnss_set_rate(gnss_dev, ev->rate);
			if (ret != 0) {
				char *msg = "Failed to set GNSS receiver data "
					    "rate!";
				nf_app_error(ERR_GNSS_CONTROLLER, ret, msg,
					     sizeof(*msg));
				return false;
			}
			current_rate = ev->rate;
		}
		return false;
	} else if (is_gnss_switch_off(eh)) {
		int ret = 0; //gnss_switch_off(gnss_dev);
		if (ret != 0) {
			char *msg = "Failed to switch OFF GNSS receiver!";
			nf_app_error(ERR_GNSS_CONTROLLER, ret, msg,
				     sizeof(*msg));
		}
		return false;
	} else if (is_gnss_switch_on(eh)) {
		int ret = 0; //gnss_switch_on(gnss_dev);
		if (ret != 0) {
			char *msg = "Failed to switch ON GNSS receiver!";
			nf_app_error(ERR_GNSS_CONTROLLER, ret, msg,
				     sizeof(*msg));
		}
		return false;
	} else if (is_gnss_set_mode_event(eh)) {
		struct gnss_set_mode_event *ev = cast_gnss_set_mode_event(eh);
		if (ev->mode != current_mode) {
			int ret = gnss_set_mode(ev->mode);
			if (ret != 0) {
				char *msg = "Failed to set GNSS receiver mode";
				nf_app_error(ERR_GNSS_CONTROLLER, ret, msg,
					     sizeof(*msg));
				return false;
			}
			current_mode = ev->mode;
		}
		return false;
	}
	return false;
}

EVENT_LISTENER(MODULE, gnss_controller_event_handler);
EVENT_SUBSCRIBE(MODULE, gnss_data_rate);
EVENT_SUBSCRIBE(MODULE, gnss_switch_off);
EVENT_SUBSCRIBE(MODULE, gnss_switch_on);
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
	LOG_DBG("resets %d", gnss_reset_count);

	if (gnss_reset_count < 1) {
		gnss_reset_count++;
		gnss_controller_reset_gnss(GNSS_RESET_MASK_HOT);
	} else if (gnss_reset_count < 2) {
		gnss_reset_count++;
		gnss_controller_reset_gnss(GNSS_RESET_MASK_WARM);
	} else if (gnss_reset_count >= 2) {
		gnss_reset_count++;
		gnss_controller_reset_gnss(GNSS_RESET_MASK_COLD);
	}

	if (gnss_reset_count >= 3) {
		LOG_DBG("OLD FIX ERROR!\n");
		char *msg = "GNSS fix extremely old!";
		nf_app_error(ERR_GNSS_CONTROLLER, -ETIMEDOUT, msg,
			     sizeof(*msg));
	}
}

/**
 * @brief Clears the reset count. Supposed to be called whenever 
 *        a new GNSS data packet is received. 
 *
 * @return None
 *
 */
static void gnss_clear_reset_count(void)
{
	gnss_reset_count = 0;
}
