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

K_SEM_DEFINE(new_data_sem, 0, 1);
K_SEM_DEFINE(last_fix_sem, 0, 1);

#define MODULE gnss_controller
LOG_MODULE_REGISTER(MODULE, CONFIG_GNSS_CONTROLLER_LOG_LEVEL);

#define MIN_GNSS_RATE CONFIG_MINIMUM_ALLOWED_GNSS_RATE

void publish_gnss_data(void);
void publish_last_fix(void);
void set_gnss_rate(enum gnss_data_rate);
static int gnss_data_update_cb(const gnss_struct_t*);
static int last_fix_update_cb(const gnss_last_fix_struct_t*);
static uint16_t current_rate;

static gnss_struct_t gnss_data_buffer;
static gnss_last_fix_struct_t last_fix_buffer;

gnss_struct_t cached_gnss_data;
gnss_last_fix_struct_t cached_last_fix;

const struct device *gnss_dev = NULL;

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
	ret = gnss_setup(gnss_dev, true);
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
	return 0;
}
K_THREAD_DEFINE(pub_gnss, STACK_SIZE,
		publish_gnss_data, NULL, NULL, NULL,
		PRIORITY, 0, 0);

K_THREAD_DEFINE(pub_fix, STACK_SIZE,
		publish_last_fix, NULL, NULL, NULL,
		PRIORITY, 0, 0);

void publish_gnss_data(){
	while(true){
//		K_SLEEP(K_SECONDS(0.1));
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

void publish_last_fix(){
	while(true){
		if(k_sem_take(&last_fix_sem, K_FOREVER) == 0){
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
		return false;
	} else if (is_gnss_switch_on(eh)) {
		return false;
	} else if (is_gnss_set_mode(eh)) {
		return false;
	}
	return false;
}

EVENT_LISTENER(MODULE, gnss_controller_event_handler);
EVENT_SUBSCRIBE(MODULE, gnss_data_rate);
EVENT_SUBSCRIBE(MODULE, gnss_switch_off);
EVENT_SUBSCRIBE(MODULE, gnss_switch_on);
EVENT_SUBSCRIBE(MODULE, gnss_set_mode);

//void set_gnss_rate(enum gnss_data_rate new_rate){
//	uint16_t *current_rate = NULL;
//	int ret, err;
//	err = gnss_get_rate(gnss_dev, current_rate);
//	if(err){
//		printk("Failed to get GNSS rate: %d\n", err);
//		return;
//	}
//	switch (new_rate){
//	case LOW:
//		ret = gnss_set_rate(gnss_dev, 1000);
//		if (ret){
//			printk("Failed to set GNSS rate to 1000ms: %d\n", ret);
//			return;
//		}
//		break;
//	case HIGH:
//		ret = gnss_set_rate(gnss_dev, 250);
//		if (ret){
//			printk("Failed to set GNSS rate to 250ms: %d\n", ret);
//			return;
//		}
//		break;
//	}
//	return;
//}