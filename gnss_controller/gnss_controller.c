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
#define UPDATE_ANO_PRIORITY 7

#define GNSS_1SEC			1000
#define GNSS_5SEC			(GNSS_1SEC * 5)
#define GNSS_10SEC			(GNSS_1SEC * 10)
#define GNSS_20SEC			(GNSS_1SEC * 20)
#define GNSS_DATA_TIMEOUT 		(GNSS_1SEC * 25)
#define UNIX_TIMESTAMP_1_JAN_2000 1451606400

K_SEM_DEFINE(new_data_sem, 0, 1);
K_SEM_DEFINE(cached_fix_sem, 1, 1);

K_SEM_DEFINE(ano_update, 0, 1);

#define MODULE gnss_controller
LOG_MODULE_REGISTER(MODULE, CONFIG_GNSS_CONTROLLER_LOG_LEVEL);

#define MIN_GNSS_RATE CONFIG_MINIMUM_ALLOWED_GNSS_RATE

_Noreturn void publish_gnss_data(void* ctx);
void set_gnss_rate(enum gnss_data_rate);
static int gnss_data_update_cb(const gnss_t*);
bool check_gnss_age(uint32_t);

static uint16_t current_rate;

static gnss_t gnss_data_buffer;
uint32_t gnss_age, ts, previous_ts;
enum gnss_mode current_mode = GNSSMODE_NOMODE;

gnss_t cached_gnss_data;

const struct device *gnss_dev = NULL;
uint8_t gnss_reset_count;
uint32_t current_time;
_Noreturn void install_fresh_ano(void);
/*
K_THREAD_DEFINE(pub_gnss, STACK_SIZE,
		publish_gnss_data, NULL, NULL, NULL,
		PRIORITY, 0, 0);
		*/

K_THREAD_STACK_DEFINE(pub_gnss_stack, NEW_GNSS_DATA_STACK_SIZE);
struct k_thread pub_gnss_thread;
bool pub_gnss_started, update_ano_started = false;

K_THREAD_STACK_DEFINE(update_ano_stack, UPDATE_ANO_STACK_SIZE);
struct k_thread update_ano_thread;

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
	if (!pub_gnss_started) {
		k_thread_create(&pub_gnss_thread, pub_gnss_stack,
				K_KERNEL_STACK_SIZEOF(pub_gnss_stack),
				(k_thread_entry_t) publish_gnss_data,
				(void*)NULL, NULL, NULL,
				NEW_GNSS_DATA_PRIORITY, 0, K_NO_WAIT);
		pub_gnss_started = true;
	}
	if (!update_ano_started) {
		k_thread_create(&update_ano_thread, update_ano_stack,
				K_KERNEL_STACK_SIZEOF(update_ano_stack),
				(k_thread_entry_t) install_fresh_ano,
				NULL, NULL, NULL,
				UPDATE_ANO_PRIORITY, 0, K_NO_WAIT);
		update_ano_started = true;
	}
//	ret = stg_clear_partition(STG_PARTITION_ANO);
//	if (ret != 0){
//		LOG_ERR("Failed to clear ANO partition!");
//	}
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

_Noreturn void publish_gnss_data(void* ctx){
	while(true){
		if(k_sem_take(&new_data_sem, K_SECONDS(GNSS_DATA_TIMEOUT)) == 0) {
			if (gnss_data_buffer.fix_ok) {
				ts = k_uptime_get_32();
				if (ts >= previous_ts){
					gnss_age = ts - previous_ts;
				} else{ //handle overflow
					gnss_age = UINT32_MAX + ts - previous_ts;
				}
				LOG_DBG("gnss fix age = age:%d, ts:%d", gnss_age,
					ts);
				if (check_gnss_age(gnss_age)) {
					k_sem_take(&ano_update, K_SECONDS(1));
					int ret = gnss_setup(gnss_dev, false);
					if (ret != 0) {
						char *msg = "Failed to set up"
							    " GNSS receiver "
							    "after reset!";
						nf_app_error(
							ERR_GNSS_CONTROLLER,
							ret, msg, sizeof(*msg));
					}
					k_sem_give(&ano_update);
				}
				previous_ts = ts;
			}

			struct gnss_data* new_data = new_gnss_data();
			new_data->gnss_data = gnss_data_buffer;
//			LOG_INF("New GNSS data received!\n");
			LOG_INF("ttff = %d", gnss_data_buffer.latest.ttff);
			LOG_INF("GNSS data: %d, %d, %d, %d, %d", gnss_data_buffer.latest.lon, gnss_data_buffer.latest.lat, gnss_data_buffer.latest.pvt_flags, gnss_data_buffer.latest.h_acc_dm, gnss_data_buffer.latest.num_sv);
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
	}
	if (is_gnss_switch_off(eh)) {
		int ret = 0; //gnss_switch_off(gnss_dev);
		if (ret != 0){
			char* msg = "Failed to switch OFF GNSS receiver!";
			nf_app_error(ERR_GNSS_CONTROLLER, ret, msg, sizeof(*msg));
		}
		return false;
	}
	if (is_gnss_switch_on(eh)) {
		int ret = 0; //gnss_switch_on(gnss_dev);
		if (ret != 0){
			char* msg = "Failed to switch ON GNSS receiver!";
			nf_app_error(ERR_GNSS_CONTROLLER, ret, msg, sizeof(*msg));
		}
		return false;
	}
	if (is_gnss_set_mode(eh)) {
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
	if (is_ano_ready(eh)){
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
EVENT_SUBSCRIBE(MODULE,ano_ready);

/**
 * @brief check whether the gnss fix is too old.
 *
 * @param[in] data Pointer to available gnss fix age.
 *
 * @return none, but GNSS receiver might be reset if the fix is too old.
 *
 *
 */
bool check_gnss_age(uint32_t gnss_age) {
	LOG_DBG("resets %d", gnss_reset_count);

	if (gnss_reset_count >= 3){
		LOG_DBG("OLD FIX ERROR!\n");
		char* msg = "GNSS fix extremely old!";
		nf_app_error(ERR_GNSS_CONTROLLER, -ETIMEDOUT, msg, sizeof(*msg));
	}

	if ((gnss_age > GNSS_5SEC) && (gnss_reset_count < 1)) {
		gnss_reset_count++;
		struct gnss_no_zone *noZone = new_gnss_no_zone();
		EVENT_SUBMIT(noZone);
		gnss_reset(gnss_dev, GNSS_RESET_MASK_HOT, GNSS_RESET_MODE_HW_IMMEDIATELY);
		return true;
		//TODO: check if gnss_setup() is required
	} else if (gnss_age > GNSS_10SEC && gnss_reset_count < 2) {
		gnss_reset_count++;
		struct gnss_no_zone *noZone = new_gnss_no_zone();
		EVENT_SUBMIT(noZone);
		gnss_reset(gnss_dev, GNSS_RESET_MASK_WARM, GNSS_RESET_MODE_HW_IMMEDIATELY);
		return true;
	} else if (gnss_age > GNSS_20SEC && gnss_reset_count >= 2) {
		gnss_reset_count++;
		struct gnss_no_zone *noZone = new_gnss_no_zone();
		EVENT_SUBMIT(noZone);
		gnss_reset(gnss_dev, GNSS_RESET_MASK_COLD, GNSS_RESET_MODE_HW_IMMEDIATELY);
		return true;
	}
	else if (gnss_age < GNSS_5SEC){
		gnss_reset_count = 0;
		return false;
	}
	return false;
}

static uint32_t ano_date_to_unixtime_midday(uint8_t year, uint8_t month,
					    uint8_t day)
{
	nf_time_t nf_time;
	nf_time.day = day;
	nf_time.month = month;
	nf_time.year = year + 2000;
	nf_time.hour = 12; //assumed per ANO specs
	nf_time.minute = nf_time.second = 0;
	return time2unix(&nf_time);
}

int install_ano_msg(GPS_UBX_MGA_ANO_t* new_ano_message)
{
	return gnss_upload_assist_data(gnss_dev, (uint8_t*)new_ano_message,
				       sizeof(GPS_UBX_MGA_ANO_t));
}

GPS_UBX_MGA_ANO_t ano_msg;
static inline int read_ano_callback(uint8_t *data, size_t len)
{
	uint32_t ano_msg_age;
	ano_msg_age =
		ano_date_to_unixtime_midday(
			ano_msg.year,
			ano_msg.month,
			ano_msg.day);
	LOG_INF("ano msg age=%d, svID =%d",
		ano_msg_age, ano_msg.svId);
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
	static bool all_ano_installed = false;
	uint32_t ano_msg_age = 0;
	int ret;
	while (true) {
		current_time = gnss_data_buffer.lastfix.unix_timestamp;
		if (current_time > UNIX_TIMESTAMP_1_JAN_2000) {
			if (!all_ano_installed) {
				//			k_sem_take(&ano_update, K_FOREVER);
				int err = stg_read_ano_data(read_ano_callback,
							    false, 1);
				if (err == 0) {
					/*check timestamp and install if fresh.*/
					ano_msg_age =
						ano_date_to_unixtime_midday(
							ano_msg.year,
							ano_msg.month,
							ano_msg.day);
					if (ano_msg_age > current_time
					    && ano_msg_age >=
						       last_installed_ano_time) {
						ret = install_ano_msg(&ano_msg);
						if (ret == 0) {
							last_installed_ano_time =
								ano_msg_age;
							LOG_INF("Installed ano "
							      "msg!");
						} else {
							LOG_INF("Failed to "
							      "install "
								"ano msg! ,%d",
								ret);
						}
						//TODO: handle failure to install ano_msg
					} else {
						LOG_INF("Finished installing "
							" all ano from flash!");
						all_ano_installed = true;
					}
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
			if (last_installed_ano_time + offset < current_time) {
				all_ano_installed = true;
				/* Time to download new ano data from the server*/
				struct start_ano_download *ev =
					new_start_ano_download();
				LOG_INF("Submitting req_ano!");
				EVENT_SUBMIT(ev);
				int err = stg_clear_partition
					(STG_PARTITION_ANO);
				if(err != 0){
					LOG_ERR("Failed to clear ano "
						"partition!");
				}
				k_sem_take(&ano_update, K_MINUTES(5));
				/* wait until ano download is complete or
				 * timeout expires. */
			}
		}
//		k_yield();
		k_sleep(K_SECONDS(1));
	}
}