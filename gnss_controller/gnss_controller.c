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

#define GNSS_1SEC			1000
#define GNSS_5SEC			(GNSS_1SEC * 5)
#define GNSS_10SEC			(GNSS_1SEC * 10)
#define GNSS_20SEC			(GNSS_1SEC * 20)
#define GNSS_DATA_TIMEOUT 		(GNSS_1SEC * 25)

K_SEM_DEFINE(new_data_sem, 0, 1);
K_SEM_DEFINE(cached_fix_sem, 1, 1);

#define MODULE gnss_controller
LOG_MODULE_REGISTER(MODULE, CONFIG_GNSS_CONTROLLER_LOG_LEVEL);

#define MIN_GNSS_RATE CONFIG_MINIMUM_ALLOWED_GNSS_RATE

_Noreturn void publish_gnss_data(void);
void set_gnss_rate(enum gnss_data_rate);
static int gnss_data_update_cb(const gnss_t*);
void check_gnss_age(uint32_t);

static uint16_t current_rate;

static gnss_t gnss_data_buffer;
uint32_t gnss_age, ts, previous_ts;
gnss_mode_t current_mode = GNSSMODE_NOMODE;

gnss_t cached_gnss_data;

const struct device *gnss_dev = NULL;
uint8_t gnss_reset_count;


int gnss_controller_init(void){
	gnss_age = 0;
	ts = 0;
	previous_ts = 0;
	gnss_reset_count = 0;
	printk("Initializing gnss controller!\n");
	gnss_dev = DEVICE_DT_GET(DT_ALIAS(gnss));
	if (gnss_dev == NULL) {
		char* msg = "Couldn't get instance of the GNSS device!";
		printk("%s", msg);
		nf_app_error(ERR_GNSS_CONTROLLER, -1, msg, sizeof(*msg));
		return -1;
	}
	int ret = gnss_set_data_cb(gnss_dev, gnss_data_update_cb);
	if(ret != 0){
		char* msg = "Failed to register data CB!";
		nf_app_error(ERR_GNSS_CONTROLLER, ret, msg, sizeof(*msg));
		return ret;
	}
	ret = gnss_setup(gnss_dev, false);
	if(ret != 0){
		char* msg = "Failed to set up GNSS receiver!";
		nf_app_error(ERR_GNSS_CONTROLLER, ret, msg, sizeof(*msg));
		return ret;
	}
	ret = gnss_get_rate(gnss_dev, &current_rate);
	if(ret != 0){
		char* msg = "Failed to get GNSS receiver data rate!";
		nf_app_error(ERR_GNSS_CONTROLLER, ret, msg, sizeof(*msg));
		return ret;
	}
	ret = 0; // gnss_get_mode(gnss_dev, &current_mode);
	if(ret != 0){
		char* msg = "Failed to get GNSS receiver mode!";
		nf_app_error(ERR_GNSS_CONTROLLER, ret, msg, sizeof(*msg));
		return ret;
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
K_THREAD_DEFINE(pub_gnss, STACK_SIZE,
		publish_gnss_data, NULL, NULL, NULL,
		PRIORITY, 0, 0);


_Noreturn void publish_gnss_data(){
	while(true){
		if(k_sem_take(&new_data_sem, K_SECONDS(25)) == 0) {
			if (gnss_data_buffer.fix_ok) {
				ts = k_uptime_get_32();
				if (ts >= previous_ts){
					gnss_age = ts - previous_ts;
				} else{ //handle overflow
					gnss_age = UINT32_MAX + ts - previous_ts;
				}
				LOG_DBG("gnss fix age = age:%d, ts:%d", gnss_age,
					ts);
				check_gnss_age(gnss_age);
				previous_ts = ts;
			}

			struct gnss_data* new_data = new_gnss_data();
			new_data->gnss_data = gnss_data_buffer;
			LOG_INF("New GNSS data received!\n");
			EVENT_SUBMIT(new_data);
			if(k_sem_take(&cached_fix_sem, K_MSEC(100)) == 0){
				cached_gnss_data = gnss_data_buffer;
				k_sem_give(&cached_fix_sem);
			} else{ /* TODO: take action if needed. */
				LOG_WRN("Failed to update cached GNSS fix!\n");
			}
		} else{
			char* msg = "GNSS data timed out!";
			nf_app_error(ERR_GNSS_CONTROLLER, -ETIMEDOUT, msg, sizeof(*msg));
			struct gnss_no_zone *noZone = new_gnss_no_zone();
			EVENT_SUBMIT(noZone);
			gnss_reset(gnss_dev, GNSS_RESET_MASK_COLD, GNSS_RESET_MODE_HW_IMMEDIATELY);
		}
	}
}

static int gnss_data_update_cb(const gnss_t* data) {
	memcpy(&gnss_data_buffer, data, sizeof(gnss_t));
	k_sem_give(&new_data_sem);
	return 0;
}

static bool gnss_controller_event_handler(const struct event_header *eh)
{
	if (is_gnss_rate(eh)) {
		struct gnss_rate *ev =
			cast_gnss_rate(eh);
			if (ev->rate >= MIN_GNSS_RATE  && current_rate != ev->rate){
			int ret = gnss_set_rate(gnss_dev, ev->rate);
			if (ret != 0){
				char* msg = "Failed to set GNSS receiver data "
					    "rate!";
				nf_app_error(ERR_GNSS_CONTROLLER, ret, msg, sizeof(*msg));
				return false;
			}
			current_rate = ev->rate;
		}
		return false;
	} else if (is_gnss_switch_off(eh)) {
		int ret = 0; //gnss_switch_off(gnss_dev);
		if (ret != 0){
			char* msg = "Failed to switch OFF GNSS receiver!";
			nf_app_error(ERR_GNSS_CONTROLLER, ret, msg, sizeof(*msg));
		}
		return false;
	} else if (is_gnss_switch_on(eh)) {
		int ret = 0; //gnss_switch_on(gnss_dev);
		if (ret != 0){
			char* msg = "Failed to switch ON GNSS receiver!";
			nf_app_error(ERR_GNSS_CONTROLLER, ret, msg, sizeof(*msg));
		}
		return false;
	} else if (is_gnss_set_mode(eh)) {
		struct gnss_set_mode *ev =
			cast_gnss_set_mode(eh);
		if (ev->mode!= current_mode){
			int ret = 0; //gnss_set_mode(gnss_dev, ev->mode);
			if (ret != 0){
				char* msg = "Failed to set GNSS receiver mode";
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
EVENT_SUBSCRIBE(MODULE, gnss_data_rate);
EVENT_SUBSCRIBE(MODULE, gnss_switch_off);
EVENT_SUBSCRIBE(MODULE, gnss_switch_on);
EVENT_SUBSCRIBE(MODULE, gnss_set_mode);

/**
 * @brief check whether the gnss fix is too old.
 *
 * @param[in] data Pointer to available gnss fix age.
 *
 * @return none, but GNSS receiver might be reset if the fix is too old.
 *
 *
 */
void check_gnss_age(uint32_t gnss_age) {
	LOG_DBG("resets %d", gnss_reset_count);

	if ((gnss_age > GNSS_5SEC) && (gnss_reset_count < 1)) {
		gnss_reset_count++;
		struct gnss_no_zone *noZone = new_gnss_no_zone();
		EVENT_SUBMIT(noZone);
		gnss_reset(gnss_dev, GNSS_RESET_MASK_HOT, GNSS_RESET_MODE_HW_IMMEDIATELY);
		//TODO: check if gnss_setup() is required
	} else if (gnss_age > GNSS_10SEC && gnss_reset_count < 2) {
		gnss_reset_count++;
		struct gnss_no_zone *noZone = new_gnss_no_zone();
		EVENT_SUBMIT(noZone);
		gnss_reset(gnss_dev, GNSS_RESET_MASK_WARM, GNSS_RESET_MODE_HW_IMMEDIATELY);
	} else if (gnss_age > GNSS_20SEC && gnss_reset_count >= 2) {
		gnss_reset_count++;
		struct gnss_no_zone *noZone = new_gnss_no_zone();
		EVENT_SUBMIT(noZone);
		gnss_reset(gnss_dev, GNSS_RESET_MASK_COLD, GNSS_RESET_MODE_HW_IMMEDIATELY);
	}
	else if (gnss_age < GNSS_5SEC){
		gnss_reset_count = 0;
	}

	if (gnss_reset_count >= 3){
		LOG_DBG("OLD FIX ERROR!\n");
		char* msg = "GNSS fix extremely old!";
		nf_app_error(ERR_GNSS_CONTROLLER, -ETIMEDOUT, msg, sizeof(*msg));
	}
}
