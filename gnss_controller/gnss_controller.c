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

#define GPS_1SEC			1000
#define GPS_5SEC			(GPS_1SEC * 5)
#define GPS_10SEC			(GPS_1SEC * 10)
#define GPS_20SEC			(GPS_1SEC * 20)

K_SEM_DEFINE(new_data_sem, 0, 1);
K_SEM_DEFINE(last_fix_sem, 0, 1);

#define MODULE gnss_controller
LOG_MODULE_REGISTER(MODULE, CONFIG_GNSS_CONTROLLER_LOG_LEVEL);

#define MIN_GNSS_RATE CONFIG_MINIMUM_ALLOWED_GNSS_RATE

_Noreturn void publish_gnss_data(void);
_Noreturn void publish_last_fix(void);
void set_gnss_rate(enum gnss_data_rate);
static int gnss_data_update_cb(const gnss_struct_t*);
static int last_fix_update_cb(const gnss_last_fix_struct_t*);
void check_gnss_age(uint32_t *);

static uint16_t current_rate;

static gnss_struct_t gnss_data_buffer;
static gnss_last_fix_struct_t last_fix_buffer;
uint32_t gnss_age, temp_age;
enum gnss_mode current_mode = GPSMODE_NOMODE;

gnss_struct_t cached_gnss_data;
gnss_last_fix_struct_t cached_last_fix;

const struct device *gnss_dev = NULL;
bool m_u8_GPSFresh;

int gnss_controller_init(void){
	printk("Initializing gnss controller!\n");
	gnss_dev = DEVICE_DT_GET(DT_ALIAS(gnss));
	if (gnss_dev == NULL) {
		char* msg = "Couldn't get instance of the GNSS device!";
		printk("%s", msg);
		nf_app_error(GPS_CONTROLLER, -1, msg, sizeof(*msg));
		return -1;
	}
	int ret = gnss_set_data_cb(gnss_dev, gnss_data_update_cb);
	if(ret != 0){
		char* msg = "Failed to register data CB!";
		nf_app_error(GPS_CONTROLLER, ret, msg, sizeof(*msg));
		return ret;
	}
	ret = gnss_set_lastfix_cb(gnss_dev, last_fix_update_cb);
	if(ret != 0){
		char* msg = "Failed to register last fix CB!";
		nf_app_error(GPS_CONTROLLER, ret, msg, sizeof(*msg));
		return ret;
	}
	ret = gnss_setup(gnss_dev, false);
	if(ret != 0){
		char* msg = "Failed to set up GNSS receiver!";
		nf_app_error(GPS_CONTROLLER, ret, msg, sizeof(*msg));
		return ret;
	}
	ret = gnss_get_rate(gnss_dev, &current_rate);
	if(ret != 0){
		char* msg = "Failed to get GNSS receiver data rate!";
		nf_app_error(GPS_CONTROLLER, ret, msg, sizeof(*msg));
		return ret;
	}
	ret = 0; // gnss_get_mode(gnss_dev, &current_mode);
	if(ret != 0){
		char* msg = "Failed to get GNSS receiver mode!";
		nf_app_error(GPS_CONTROLLER, ret, msg, sizeof(*msg));
		return ret;
	}
	while (true){
		k_sleep(K_SECONDS(10));
		LOG_WRN("Attempting GNSS stop!\n");
		ret = gnss_reset(gnss_dev, GNSS_RESET_MASK_COLD,
				 GNSS_RESET_MODE_GNSS_STOP);
		if (ret != 0){
			LOG_WRN("Finished GNSS stop!\n");
		}

		k_sleep(K_SECONDS(20));
		LOG_WRN("Attempting GNSS start!\n");
		ret = gnss_reset(gnss_dev, GNSS_RESET_MASK_COLD,
				 GNSS_RESET_MODE_GNSS_START);
		if (ret != 0){
			LOG_WRN("Finished GNSS start!\n");
		}
	}

	return 0;
}
K_THREAD_DEFINE(pub_gnss, STACK_SIZE,
		publish_gnss_data, NULL, NULL, NULL,
		PRIORITY, 0, 0);

K_THREAD_DEFINE(pub_fix, STACK_SIZE,
		publish_last_fix, NULL, NULL, NULL,
		PRIORITY, 0, 0);


_Noreturn void publish_gnss_data(){
	while(true){
		if(k_sem_take(&new_data_sem, K_FOREVER) == 0){
			struct new_gnss_data* new_data = new_new_gnss_data();
			new_data->gnss_data = gnss_data_buffer;
			/* TODO: protect cached data */
				cached_gnss_data = gnss_data_buffer;
			EVENT_SUBMIT(new_data);
			LOG_WRN("New GNSS data received!\n");
		}
	}
}

_Noreturn void publish_last_fix(){
	while(true){
		if(k_sem_take(&last_fix_sem, K_FOREVER) == 0){
			temp_age = k_uptime_get_32();
			if (temp_age > gnss_age){
				gnss_age = temp_age - gnss_age;
			} else{ //handle overflow
				gnss_age = UINT32_MAX + temp_age - gnss_age;
			}
			check_gnss_age(&gnss_age);
			struct new_gnss_fix* new_fix = new_new_gnss_fix();
			new_fix->fix = last_fix_buffer;
			/* TODO: protect cached data */
			cached_last_fix = last_fix_buffer;
			EVENT_SUBMIT(new_fix);
			LOG_WRN("New GNSS fix received!\n");
		}
	}
}

static int gnss_data_update_cb(const gnss_struct_t* data) {
	memcpy(&gnss_data_buffer, data, sizeof(gnss_struct_t));
	k_sem_give(&new_data_sem);
	return 0;
}

static int last_fix_update_cb(const gnss_last_fix_struct_t* fix) {
	memcpy(&last_fix_buffer, fix, sizeof(gnss_last_fix_struct_t));
	k_sem_give(&last_fix_sem);
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
				nf_app_error(GPS_CONTROLLER, ret, msg, sizeof(*msg));
				return false;
			}
			current_rate = ev->rate;
		}
		return false;
	} else if (is_gnss_switch_off(eh)) {
		int ret = 0; //gnss_switch_off(gnss_dev);
		if (ret != 0){
			char* msg = "Failed to switch OFF GNSS receiver!";
			nf_app_error(GPS_CONTROLLER, ret, msg, sizeof(*msg));
		}
		return false;
	} else if (is_gnss_switch_on(eh)) {
		int ret = 0; //gnss_switch_on(gnss_dev);
		if (ret != 0){
			char* msg = "Failed to switch ON GNSS receiver!";
			nf_app_error(GPS_CONTROLLER, ret, msg, sizeof(*msg));
		}
		return false;
	} else if (is_gnss_set_mode(eh)) {
		struct gnss_set_mode *ev =
			cast_gnss_set_mode(eh);
		if (ev->mode!= current_mode){
			int ret = 0; //gnss_set_mode(gnss_dev, ev->mode);
			if (ret != 0){
				char* msg = "Failed to set GNSS receiver mode";
				nf_app_error(GPS_CONTROLLER, ret, msg, sizeof(*msg));
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
void check_gnss_age(uint32_t *gnss_age) {
	static uint8_t myGPSResetDone;
	if (m_u8_GPSFresh) {
		myGPSResetDone = 0;
	}

	if ((*gnss_age > GPS_5SEC) && (myGPSResetDone < 1)) {
		myGPSResetDone++;
		struct gnss_no_zone *noZone = new_gnss_no_zone();
		EVENT_SUBMIT(noZone);
		gnss_reset(gnss_dev, GNSS_RESET_MASK_HOT, GNSS_RESET_MODE_HW_IMMEDIATELY);
		//TODO increase counter to report how often this occurs
	} else if (*gnss_age > GPS_10SEC && myGPSResetDone < 2) {
		myGPSResetDone++;
		struct gnss_no_zone *noZone = new_gnss_no_zone();
		EVENT_SUBMIT(noZone);
		gnss_reset(gnss_dev, GNSS_RESET_MASK_WARM, GNSS_RESET_MODE_HW_IMMEDIATELY);
	} else if (*gnss_age > GPS_20SEC && myGPSResetDone < 3) {
		myGPSResetDone++;
		struct gnss_no_zone *noZone = new_gnss_no_zone();
		EVENT_SUBMIT(noZone);
		gnss_reset(gnss_dev, GNSS_RESET_MASK_COLD, GNSS_RESET_MODE_HW_IMMEDIATELY);
	}
	else{
		m_u8_GPSFresh = true;
	}

	if (myGPSResetDone > 0) {
		m_u8_GPSFresh = false;
	}
}
