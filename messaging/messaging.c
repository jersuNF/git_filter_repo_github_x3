/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include <stdlib.h>
#include <string.h>
#include "messaging_module_events.h"
#include "messaging.h"
#include <logging/log.h>
#include "ble_ctrl_event.h"
#include "ble_data_event.h"
#include "ble_cmd_event.h"
#include "nofence_service.h"
#include "pwr_event.h"
#include "env_sensor_event.h"
#include "pwr_module.h"
#include "watchdog_event.h"
#include "cellular_controller_events.h"
#include "gnss_controller_events.h"
#include "request_events.h"
#include "nf_version.h"
#include "UBX.h"
#include "unixTime.h"
#include "error_event.h"

#include "nf_crc16.h"
#include "storage_event.h"

#include <date_time.h>

#include "storage.h"
#include "stg_config.h"
#include "movement_events.h"
#include "pasture_structure.h"
#include "fw_upgrade_events.h"
#include "sound_event.h"
#include "histogram_events.h"
#include <sys/sys_heap.h>
#include "amc_const.h"

#define MODULE messaging
LOG_MODULE_REGISTER(MODULE, CONFIG_MESSAGING_LOG_LEVEL);

#define DOWNLOAD_COMPLETE 255
#define GPS_UBX_NAV_PVT_VALID_HEADVEH_MASK 0x20
#define SECONDS_IN_THREE_DAYS 259200
#define MSECCONDS_PER_HOUR 3600000

#define BYTESWAP16(x) (((x) << 8) | ((x) >> 8))

#define EMPTY_FENCE_CRC 0xFFFF
static pasture_t pasture_temp;
static uint8_t cached_fences_counter = 0;
static uint32_t cached_msss = 0;
static uint32_t cached_ttff = 0;

/** @todo This should be fetched from storage */
static gnss_mode_t cached_gnss_mode = GNSSMODE_NOMODE;

atomic_t cached_batt = ATOMIC_INIT(0);
static uint32_t cached_chrg;
atomic_t cached_temp = ATOMIC_INIT(0);
atomic_t cached_press = ATOMIC_INIT(0);
atomic_t cached_hum = ATOMIC_INIT(0);
atomic_t cached_warning_duration = ATOMIC_INIT(0);
atomic_t cached_dist_zap = ATOMIC_INIT(0);
atomic_t cached_dist_warn = ATOMIC_INIT(0);
atomic_t cached_dist_correction_start = ATOMIC_INIT(0);
atomic_t cached_dist_correction_end = ATOMIC_INIT(0);

K_SEM_DEFINE(cache_ready_sem, 0, 1);
K_SEM_DEFINE(cache_lock_sem, 1, 1);
K_SEM_DEFINE(send_out_ack, 0, 1);
K_SEM_DEFINE(connection_ready, 0, 1);
K_SEM_DEFINE(sem_release_tx_thread, 0, 1);

static collar_state_struct_t current_state;
static gnss_last_fix_struct_t cached_fix;

static bool fota_reset = true;
static bool block_fota_request = false;

typedef enum {
	COLLAR_MODE,
	COLLAR_STATUS,
	FENCE_STATUS,
	FENCE_VERSION,
	/** @todo Not written to storage, should it? -> FLASH_ERASE_COUNT,*/
	ZAP_COUNT,
	GNSS_STRUCT,
	GSM_INFO,
	CACHED_READY_END_OF_LIST
} cached_and_ready_enum;

/* Used to check if we've cached what is requried before we perform the 
 * INITIAL, FIRST poll request to server */
static int cached_and_ready_reg[CACHED_READY_END_OF_LIST];

static int rat, mnc, rssi, min_rssi, max_rssi;
static uint8_t ccid[20] = "\0";

static uint8_t expected_ano_frame, new_ano_in_progress;
static bool first_ano_frame;

/* Time since the server updated the date time in seconds. */
static atomic_t server_timestamp_sec = ATOMIC_INIT(0);

static uint32_t serial_id = 0;

/* Bitflags indicating whether collar states are initialized (See collar_state_flags,
 * set_initial_collar_state_flag and has_initial_collar_states) */
static uint8_t m_initial_collar_state_flags = 0;
enum collar_state_flags {
    COLLAR_MODE_FLAG = 0,
    COLLAR_STATUS_FLAG,
    FENCE_STATUS_FLAG,
    BATTERY_LVL_FLAG,
    COLLAR_STATE_FLAG_CNT,
};
static inline void set_initial_collar_state_flag(uint8_t aFlag);
static inline bool has_initial_collar_states();

void build_poll_request(NofenceMessage *);
void fence_download(uint8_t);
int8_t request_ano_frame(uint16_t, uint16_t);
void ano_download(uint16_t, uint16_t);
void proto_InitHeader(NofenceMessage *);
void process_poll_response(NofenceMessage *);
void process_upgrade_request(VersionInfoFW *);
uint8_t process_fence_msg(FenceDefinitionResponse *);
uint8_t process_ano_msg(UbxAnoReply *);

int encode_and_send_message(NofenceMessage *);
int encode_and_store_message(NofenceMessage *);
int send_binary_message(uint8_t *, size_t);
static int send_all_stored_messages(void);

static void proto_get_last_known_date_pos(gnss_last_fix_struct_t *gpsLastFix, _DatePos *pos);
bool proto_has_last_known_date_pos(const gnss_last_fix_struct_t *);
static uint32_t ano_date_to_unixtime_midday(uint8_t, uint8_t, uint8_t);

void messaging_rx_thread_fn(void);
void messaging_tx_thread_fn(void);

static bool m_transfer_boot_params = true;
static bool m_confirm_acc_limits, m_confirm_ble_key;

K_MUTEX_DEFINE(send_binary_mutex);
K_MUTEX_DEFINE(read_flash_mutex);
static bool reboot_scheduled;

K_MSGQ_DEFINE(ble_cmd_msgq, sizeof(struct ble_cmd_event), CONFIG_MSGQ_BLE_CMD_SIZE,
	      4 /* Byte alignment */);
K_MSGQ_DEFINE(lte_proto_msgq, sizeof(struct messaging_proto_out_event), CONFIG_MSGQ_LTE_PROTO_SIZE,
	      4 /* Byte alignment */);

#define NUM_MSGQ_EVENTS 4
struct k_poll_event msgq_events[NUM_MSGQ_EVENTS] = {
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY,
					&ble_cmd_msgq, 0),
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY,
					&lte_proto_msgq, 0),
};

/* Messaging work queue items */
static struct k_work_q message_q;
struct k_work_delayable modem_poll_work;
struct k_work_delayable log_periodic_work;
struct k_work_delayable data_request_work;
struct k_work_delayable log_status_message_work;
struct k_work_delayable process_escape_work;
struct k_work_delayable process_zap_work;
struct k_work_delayable process_warning_work;
struct k_work_delayable process_warning_correction_start_work;
struct k_work_delayable process_warning_correction_end_work;
struct k_work_delayable log_send_work;

struct fence_def_update {
	struct k_work_delayable work;
	int version;
	int request_frame;
}m_fence_update_req;

atomic_t poll_period_seconds = ATOMIC_INIT(15*60);
atomic_t log_period_minutes = ATOMIC_INIT(30);

/* Messaging Rx thread */
K_THREAD_DEFINE(messaging_rx_thread, CONFIG_MESSAGING_THREAD_STACK_SIZE, messaging_rx_thread_fn,
		NULL, NULL, NULL, K_PRIO_COOP(CONFIG_MESSAGING_THREAD_PRIORITY), 0, 0);

/* Messaging Tx thread */
K_THREAD_DEFINE(messaging_tx_thread, CONFIG_MESSAGING_THREAD_STACK_SIZE, messaging_tx_thread_fn,
		NULL, NULL, NULL, K_PRIO_COOP(CONFIG_MESSAGING_THREAD_PRIORITY), 0, 0);

/* Messaging work queue */
K_KERNEL_STACK_DEFINE(messaging_send_thread, CONFIG_MESSAGING_SEND_THREAD_STACK_SIZE);

typedef enum {
	IDLE = 0 /* Tx thread is idle */,
	POLL_REQ /* Send periodic poll request */,
	LOG_MSG /* Send stored log messages */,
	FENCE_REQ /* Send a fence update request */
	/* Add additional states here (ANO, diagnostic etc)... */
}messaging_tx_type_t;

atomic_t m_halt_data_transfer = ATOMIC_INIT(0);
atomic_t m_message_tx_type = ATOMIC_INIT(0);

static int set_tx_state_ready(messaging_tx_type_t tx_type);


/**
 * @brief Builds SEQ messages (1 and 2) with the latest data and store them to external storage.
 */
static void build_log_message()
{
	int err;

	/* Fetch histogram data */
	struct save_histogram *histogram_snapshot = new_save_histogram();
	EVENT_SUBMIT(histogram_snapshot);

	collar_histogram histogram;
	err = k_msgq_get(&histogram_msgq, &histogram, K_SECONDS(10));
	if (err) {
		LOG_ERR("Timeout on waiting for histogram (%d)", err);
		nf_app_error(ERR_MESSAGING, err, NULL, 0);
		return;
	}

	/* Initialize generic seq message */
	NofenceMessage seq_msg;
	proto_InitHeader(&seq_msg);

	/* Build seq 1 message */
	seq_msg.which_m = NofenceMessage_seq_msg_tag;
	seq_msg.m.seq_msg.has_xPOS_QC_MMM = true;
	memcpy(&seq_msg.m.seq_msg.xPOS_QC_MMM, &histogram.qc_baro_gps_max_mean_min,
	       sizeof(histogram.qc_baro_gps_max_mean_min));
	seq_msg.m.seq_msg.has_usBatteryVoltage = true;
	seq_msg.m.seq_msg.usBatteryVoltage = (uint16_t)atomic_get(&cached_batt);
	seq_msg.m.seq_msg.has_usChargeMah = true;
	seq_msg.m.seq_msg.usChargeMah = (uint16_t)(cached_chrg * CONFIG_CHARGING_POLLER_WORK_MSEC /
						   MSECCONDS_PER_HOUR);
	/* TODO: Consider using a higher time resolution for more accurate integration */
	cached_chrg = 0;
	seq_msg.m.seq_msg.has_xGprsRssi = true;
	seq_msg.m.seq_msg.xGprsRssi.ucMaxRSSI = (uint8_t)max_rssi;
	seq_msg.m.seq_msg.xGprsRssi.ucMinRSSI = (uint8_t)min_rssi;
	seq_msg.m.seq_msg.has_xHistogramCurrentProfile = true;
	seq_msg.m.seq_msg.has_xHistogramZone = true;
	seq_msg.m.seq_msg.has_xHistogramAnimalBehave = true;
	memcpy(&seq_msg.m.seq_msg.xHistogramAnimalBehave, &histogram.animal_behave,
	       sizeof(histogram.animal_behave));
	memcpy(&seq_msg.m.seq_msg.xHistogramCurrentProfile, &histogram.current_profile,
	       sizeof(histogram.current_profile));
	memcpy(&seq_msg.m.seq_msg.xHistogramZone, &histogram.in_zone, sizeof(histogram.in_zone));

	/* Store Seq 1 message to non-volatile storage */
	err = encode_and_store_message(&seq_msg);
	if (err != 0) {
		LOG_ERR("Failed to encode and save sequence message 1");
		nf_app_error(ERR_MESSAGING, err, NULL, 0);
		return;
	}

	/* Build seq 2 message */
	seq_msg.which_m = NofenceMessage_seq_msg_2_tag;
	seq_msg.m.seq_msg_2.has_bme280 = true;
	seq_msg.m.seq_msg_2.bme280.ulPressure = (uint32_t)atomic_get(&cached_press);
	seq_msg.m.seq_msg_2.bme280.ulTemperature = (uint32_t)atomic_get(&cached_temp);
	seq_msg.m.seq_msg_2.bme280.ulHumidity = (uint32_t)atomic_get(&cached_hum);
	seq_msg.m.seq_msg_2.has_xBatteryQc = true;
	seq_msg.m.seq_msg_2.xBatteryQc.usVbattMax = histogram.qc_battery.usVbattMax;
	seq_msg.m.seq_msg_2.xBatteryQc.usVbattMin = histogram.qc_battery.usVbattMin;
	seq_msg.m.seq_msg_2.xBatteryQc.usTemperature = (uint16_t)atomic_get(&cached_temp);
	seq_msg.m.seq_msg_2.has_xGnssModeCounts = true;
	memcpy(&seq_msg.m.seq_msg_2.xGnssModeCounts,&histogram.gnss_modes,sizeof(seq_msg.m.seq_msg_2.xGnssModeCounts));


	/* Store Seq 2 message to non-volatile storage */
	err = encode_and_store_message(&seq_msg);
	if (err != 0) {
		LOG_ERR("Failed to encode and save sequence message 2");
		nf_app_error(ERR_MESSAGING, err, NULL, 0);
		return;
	}
}

/**
 * @brief Callback passed to the storage controller to send log messages stored to external flash.
 * @param data Encoded log messages read from storage.
 * @param len Length of the encoded log message read from storage.
 * @return Returns 0 if successfull, otherwise negative error code.
 */
int read_and_send_log_data_cb(uint8_t *data, size_t len)
{
	/* Only send log data stored to flash if not halted by some other process, e.g. a pending
	 * FOTA. Retuning an error from this callback will abort the FCB walk in the storage
	 * controller untill log data trafic is reinstated. */
	if (atomic_get(&m_halt_data_transfer) == true) {
		LOG_DBG("Unable to send log data, data transfer is currently halted");
		return -EBUSY;
	}
	LOG_DBG("Send log message fetched from flash");

	/* Fetch the length from the two first bytes */
	uint16_t new_len = *(uint16_t*) &data[0];

	int err = send_binary_message(data, new_len);
	if (err) {
		LOG_ERR("Error sending binary message for log data (%d)", err);
		nf_app_error(ERR_MESSAGING, err, NULL, 0);
	}
	return err;
}

/**
 * @brief Sends all log messages stored to external flash, see read_and_send_log_data_cb.
 * @return Returns 0 if successfull, otherwise negative error code.
 */
static int send_all_stored_messages(void)
{
	k_mutex_lock(&read_flash_mutex, K_NO_WAIT);
	if (read_flash_mutex.lock_count == 1) {
		/*Read and send out all the log data if any.*/
		int err = stg_read_log_data(read_and_send_log_data_cb, 0);
		if (err && err != -ENODATA) {
			k_mutex_unlock(&read_flash_mutex);
			LOG_ERR("stg_read_log_data error: %i", err);
			return err;
		} else if (err == -ENODATA) {
			LOG_INF("No log data available on flash for sending.");
		}

		/* If all entries has been consumed, empty storage and we HAVE data on the
		 * partition.*/
		if (stg_log_pointing_to_last()) {
			err = stg_clear_partition(STG_PARTITION_LOG);
			if (err) {
				LOG_ERR("Error clearing FCB storage for LOG %i", err);
				k_mutex_unlock(&read_flash_mutex);
				return err;
			} else {
				LOG_INF("Emptied LOG partition data as we have read everything.");
			}
		}
		k_mutex_unlock(&read_flash_mutex);
		return 0;
	} else {
		k_mutex_unlock(&read_flash_mutex);
		return -ETIMEDOUT;
	}
}

/**
 * @brief Work item handler for "log_periodic_work". Builds seq messages and store them to
 * external flash, and schedules an immediate send.
 * Rescheduled at regular interval as set by "log_period_minutes".
 */
void log_data_periodic_fn()
{
	int ret;
	ret = k_work_reschedule_for_queue(&message_q, &log_periodic_work,
					  K_MINUTES(atomic_get(&log_period_minutes)));
	if (ret < 0) {
		LOG_ERR("Failed to reschedule periodic seq messages!");
	}

	build_log_message();
	LOG_DBG("SEQ messages stored to flash");

	if (m_transfer_boot_params) {
		/* Do not send SEQ message before startup poll request is sent to server */
		return;
	}

	/* Schedule work to send log messages immediately */
	ret = k_work_reschedule_for_queue(&message_q, &log_send_work, K_NO_WAIT);
	if (ret < 0) {
		LOG_ERR("Failed to schedule a send of periodic seq messages!");
	}
}

/**
 * @brief Work item handler for "modem_poll_work". Builds a poll request and schedules an
 * immediate send.
 * Rescheduled at regular interval as set by "poll_period_minutes".
 */
void modem_poll_work_fn()
{
	/* Reschedule periodic poll request */
	int ret = k_work_reschedule_for_queue(&message_q, &modem_poll_work,
					      K_SECONDS(atomic_get(&poll_period_seconds)));
	if (ret < 0) {
		LOG_ERR("Failed to reschedule periodic poll request!");
	}

	/* Attempt to send poll request immediately */
	ret = set_tx_state_ready(POLL_REQ);
	if (ret != 0) {
		LOG_DBG("Periodic poll failed, reschedule, error %d", ret);
		/* Tx Thread busy, reschedule in 1 minute */
		k_work_reschedule_for_queue(&message_q, &modem_poll_work, K_SECONDS(15));
	}
}

/**
 * @brief Work item handler for "log_send_work". Initiate and sets Tx ready. Reschedules the
 * sending of log data if Tx thread is busy.
 */
void log_send_work_fn()
{
	static uint8_t retry_cnt = 0;

	int err = set_tx_state_ready(LOG_MSG);
	if ((err != 0) && (err != -EACCES) && (retry_cnt < 2)) {
		LOG_DBG("Unable to schedule log messages, Tx thread not ready- rescheduling");
		err = k_work_reschedule_for_queue(&message_q, &log_send_work, K_SECONDS(15));
		if (err < 0) {
			LOG_ERR("Failed to reschedule work");
		}
		retry_cnt++;
	} else if ((err != 0) && ((retry_cnt >= 2) || (err == -EACCES))) {
		LOG_DBG("Unable to send log message, exhausted retry attempts or logs halted");
		retry_cnt = 0;
	}
	return;
}

/**
 * @brief Work item handler for "process_zap_work". Builds a zap status message and schedules
 * an immediate send.
 */
static void log_zap_message_work_fn()
{
	NofenceMessage msg;
	proto_InitHeader(&msg);
	msg.which_m = NofenceMessage_client_zap_message_tag;
	msg.m.client_zap_message.has_sFenceDist = true;
	msg.m.client_zap_message.sFenceDist = (int16_t)atomic_get(&cached_dist_zap);
	proto_get_last_known_date_pos(&cached_fix, &msg.m.client_zap_message.xDatePos);
	msg.m.client_zap_message.ucReaction = 0;
	msg.m.client_zap_message.usReactionDuration = 0;

	int err = encode_and_store_message(&msg);
	if (err != 0) {
		LOG_ERR("Failed to encode and store AMC correction ZAP message");
		return;
	}
	LOG_DBG("AMC Correction ZAP message stored to flash, scheduling immediate send");

	/* Schedule work to send log messages immediately */
	err = k_work_reschedule_for_queue(&message_q, &log_send_work, K_NO_WAIT);
	if (err < 0) {
		LOG_ERR("Failed to reschedule work");
	}
}

/**
 * @brief Work item handler for "process_escape_work". Builds an animal escaped status message
 * and schedules an immediate send.
 */
static void log_animal_escaped_work_fn()
{
	NofenceMessage msg;
	proto_InitHeader(&msg);
	msg.which_m = NofenceMessage_status_msg_tag;
	msg.m.status_msg.has_datePos = true;
	proto_get_last_known_date_pos(&cached_fix, &msg.m.status_msg.datePos);
	msg.m.status_msg.eMode = current_state.collar_mode;
	msg.m.status_msg.eReason = Reason_WARNSTOPREASON_ESCAPED;
	msg.m.status_msg.eCollarStatus = current_state.collar_status;
	msg.m.status_msg.eFenceStatus = current_state.fence_status;
	msg.m.status_msg.usBatteryVoltage = (uint16_t)atomic_get(&cached_batt);
	msg.m.status_msg.has_ucGpsMode = true;
	msg.m.status_msg.ucGpsMode = (uint8_t)cached_gnss_mode;

	int err = encode_and_store_message(&msg);
	if (err) {
		LOG_ERR("Failed to encode and store AMC escaped message");
		return;
	}
	LOG_DBG("AMC Escaped message stored to flash, scheduling immediate send");

	/* Schedule work to send log messages immediately */
	err = k_work_reschedule_for_queue(&message_q, &log_send_work, K_NO_WAIT);
	if (err < 0) {
		LOG_ERR("Failed to reschedule work");
	}
}

/**
 * @brief Work item handler for "log_status_message_work". Builds a status message and stores it to
 * external flash, and schedules an immediate send. Status messages are sent for updates to collar
 * states such as Fence Status, Collar Mode and Collar Status.
 */
static void log_status_message_fn()
{
	if (!has_initial_collar_states()) {
		return;
	}

	NofenceMessage msg;
	proto_InitHeader(&msg);
	msg.which_m = NofenceMessage_status_msg_tag;
	msg.m.status_msg.has_datePos = proto_has_last_known_date_pos(&cached_fix);
	proto_get_last_known_date_pos(&cached_fix, &msg.m.status_msg.datePos);
	msg.m.status_msg.eMode = current_state.collar_mode;
	msg.m.status_msg.eReason = Reason_NOREASON;
	msg.m.status_msg.eCollarStatus = current_state.collar_status;
	msg.m.status_msg.eFenceStatus = current_state.fence_status;
	msg.m.status_msg.usBatteryVoltage = (uint16_t)atomic_get(&cached_batt);
	msg.m.status_msg.has_ucGpsMode = true;
	msg.m.status_msg.ucGpsMode = (uint8_t)cached_gnss_mode;

	int err = encode_and_store_message(&msg);
	if (err) {
		LOG_ERR("Failed to encode and store AMC status message");
		return;
	}
	LOG_DBG("AMC Status message stored to flash, scheduling immediate send");

	/* Schedule work to send log messages immediately */
	err = k_work_reschedule_for_queue(&message_q, &log_send_work, K_NO_WAIT);
	if (err < 0) {
		LOG_ERR("Failed to reschedule work");
	}
}

/**
 * @brief Work item handler for "process_warning_work". Builds a correction warning message
 * and stores it to external flash.
 */
static void log_warning_work_fn()
{
	NofenceMessage msg;
	proto_InitHeader(&msg);
	msg.which_m = NofenceMessage_client_warning_message_tag;
	msg.m.client_warning_message.has_sFenceDist = true;
	msg.m.client_warning_message.sFenceDist = (int16_t)atomic_get(&cached_dist_warn);
	msg.m.client_warning_message.usDuration = atomic_get(&cached_warning_duration);
	proto_get_last_known_date_pos(&cached_fix, &msg.m.client_zap_message.xDatePos);

	int err = encode_and_store_message(&msg);
	if (err != 0) {
		LOG_ERR("Failed to encode and store AMC correction warning message");
		return;
	}
	LOG_DBG("AMC Correction warning message stored to flash");
}

/**
 * @brief Work item handler for "process_warning_correction_start_work". Builds a correction start
 * message and stores it to external flash.
 */
static void log_correction_start_work_fn()
{
	NofenceMessage msg;
	proto_InitHeader(&msg);
	msg.which_m = NofenceMessage_client_correction_start_message_tag;
	msg.m.client_correction_start_message.has_sFenceDist = true;
	msg.m.client_correction_start_message.sFenceDist =
		(int16_t)atomic_get(&cached_dist_correction_start);
	proto_get_last_known_date_pos(&cached_fix, &msg.m.client_correction_start_message.xDatePos);

	int err = encode_and_store_message(&msg);
	if (err) {
		LOG_ERR("Failed to encode and store AMC correction start message");
		return;
	}
	LOG_DBG("AMC Correction start message stored to flash");
}

/**
 * @brief Work item handler for "process_warning_correction_end_work". Builds a correction end
 * message and stores it to external flash.
 */
static void log_correction_end_work_fn()
{
	NofenceMessage msg;
	proto_InitHeader(&msg);
	msg.which_m = NofenceMessage_client_correction_end_message_tag;
	msg.m.client_correction_end_message.has_sFenceDist = true;
	msg.m.client_correction_end_message.sFenceDist =
		(int16_t)atomic_get(&cached_dist_correction_end);
	proto_get_last_known_date_pos(&cached_fix, &msg.m.client_correction_end_message.xDatePos);

	int err = encode_and_store_message(&msg);
	if (err) {
		LOG_ERR("Failed to encode and store AMC correction end message");
		return;
	}
	LOG_DBG("AMC Correction end message stored to flash");
}

/**
 * @brief Work item handler for "m_fence_update_req.work". Schedules an immediate send of a fence
 * update request message.
 * @param item Pointer to work item (currently unused).
 */
void fence_update_req_fn(struct k_work *item)
{
	static int retry_cnt = 0;

	int ret = set_tx_state_ready(FENCE_REQ);
	if ((ret != 0) && (retry_cnt < 2)) {
		LOG_WRN("Unable to schedule fence update req., Tx thread not ready, rescheduling");
		ret = k_work_reschedule_for_queue(&message_q, &m_fence_update_req.work,
						  K_SECONDS(15));
		if (ret < 0) {
			LOG_ERR("Failed to reschedule work");
		}
		retry_cnt++;
		return;
	} else if ((ret != 0) && (retry_cnt >= 2)) {
		LOG_ERR("Unable to schedule fence update req., exhausted retry attempts");
		retry_cnt = 0;
	}
	return;
}

/**
 * @brief Function that sets Tx type and starts the Tx sending sequence, if ready. Outgoing
 * messages from the messaging module are handled in a first come first serve manner- all though
 * all log messages stored to flash are sent for each instance of LOG_MSG.
 * @param tx_state Tx message type.
 * @param send_now Flag indicating whether to send message now or not.
 * @return Returns 0 if successfull, otherwise negative error code.
 */
static int set_tx_state_ready(messaging_tx_type_t tx_type)
{
	int state = atomic_get(&m_message_tx_type);
	if (state != IDLE) {
		/* Tx thread busy sending something else */
		if ((tx_type == POLL_REQ) && (state == LOG_MSG)) {
			/* Sending log messages always starts with a poll request- no need to send
			 * additional poll */
			return 0;
		}
		return -EBUSY;
	}
	if ((tx_type != POLL_REQ) && (atomic_get(&m_halt_data_transfer) != false)) {
		/* Unable to send log messages as log data transfer is currently halted */
		return -EACCES;
	}
	state = (int)tx_type;
	atomic_set(&m_message_tx_type, state);

	/* Give semaphore to unblock messaging Tx thread and start sending sequence */
	k_sem_give(&sem_release_tx_thread);
	return 0;
}

/**
 * @brief Messaging module Tx thread.
 */
void messaging_tx_thread_fn(void)
{
        static int err = 0;
        static int64_t m_last_poll_req_timestamp_ms = 0;
	while(true) {
		/* Block Tx thread untill semaphore is given */
		if (k_sem_take(&sem_release_tx_thread, K_FOREVER) == 0) {
			err = 0;
			messaging_tx_type_t tx_type = atomic_get(&m_message_tx_type);

                        /* POLL REQUEST */
			if ((tx_type == POLL_REQ) || (m_last_poll_req_timestamp_ms == 0) ||
			    ((tx_type == LOG_MSG) &&
			    ((k_uptime_get() - m_last_poll_req_timestamp_ms) >= 60000))) {
				if (k_sem_take(&cache_ready_sem, K_SECONDS(60)) != 0) {
					LOG_WRN("Cached semaphore not ready, Sending what we have");
				}
				k_sem_give(&cache_ready_sem);

                                err = k_sem_take(&cache_lock_sem, K_SECONDS(1));
                                if (err == 0) {
					NofenceMessage poll_msg;
					build_poll_request(&poll_msg);
					k_sem_give(&cache_lock_sem);

					err = encode_and_send_message(&poll_msg);
					if (err == 0) {
                                                /* Store poll req. timestamp to avoid sending an
                                                 * excessive amount of poll requests */
                                                m_last_poll_req_timestamp_ms = k_uptime_get();
					}
                                }
                                /* Poll request error handler,
                                 * Note! Consider notifying sender, leaving error handling to src */
                                if (err != 0) {
                                        LOG_WRN("Failed to send poll request, rescheduled");
                                        if (tx_type == POLL_REQ) {
                                                int ret = k_work_reschedule_for_queue(&message_q,
								&modem_poll_work, K_MINUTES(1));
						if (ret < 0) {
							LOG_ERR("Failed to reschedule work");
						}
                                        }
                                }
			}

                        /* LOG MESSAGES */
			if ((tx_type == LOG_MSG) && (err == 0) &&
			    (atomic_get(&m_halt_data_transfer) == false)) {
				/* Sending, all stored log messages are already proto encoded */
				err = send_all_stored_messages();
                                /* Log message error handler,
                                 * Note! Consider notifying sender, leaving error handling to src */
				if (err != 0) {
					LOG_WRN("Failed to send log messages");
				}
			}

                        /* FENCE DEFINITION REQUEST */
			if ((tx_type == FENCE_REQ) &&
			    (atomic_get(&m_halt_data_transfer) == false)) {
				NofenceMessage fence_req;
				proto_InitHeader(&fence_req);
				fence_req.which_m = NofenceMessage_fence_definition_req_tag;
				fence_req.m.fence_definition_req.ulFenceDefVersion =
					m_fence_update_req.version;
				fence_req.m.fence_definition_req.ucFrameNumber =
					m_fence_update_req.request_frame;

				err = encode_and_send_message(&fence_req);
                                /* Fence def. request error handler,
                                 * Note! Consider notifying sender, leaving error handling to src */
				if (err != 0) {
					LOG_WRN("Failed to send fence update request");
				}
			}

			/* Add additional message types here (system diagnostics, ANO data etc).. */

			/* Reset Tx thread */
			atomic_set(&m_message_tx_type, IDLE);
		} else {
			LOG_WRN("Tx thread semaphore returned unexpectedly");
			k_sem_reset(&sem_release_tx_thread);
		}
	}
	LOG_ERR("Messaging Tx Thread exited unexpectedly");
}

/**
 * @brief Work item handler for "data_request_work". Sends a request for environment data, such
 * as temperatre, pressure and humidity. Rescheduled at a regular interval.
 */
void data_request_work_fn()
{
	LOG_INF("Periodic request of environment data");
	int err = k_work_reschedule_for_queue(&message_q, &data_request_work, K_MINUTES(1));
	if (err < 0) {
		LOG_ERR("Failed to reschedule work");
	}

	/* Request of temp, press, humidity */
	struct request_env_sensor_event *ev_env = new_request_env_sensor_event();
	EVENT_SUBMIT(ev_env);
}

/**
 * @brief Updates the cache registers.
 * @param index The index of the cache register to update, see cached_and_ready_enum.
 */
static void update_cache_reg(cached_and_ready_enum index)
{
	cached_and_ready_reg[index] = 1;
	for (int i = 0; i < CACHED_READY_END_OF_LIST; i++) {
		if (cached_and_ready_reg[i] == 0) {
			return;
		}
	}
	k_sem_give(&cache_ready_sem);
}

/**
 * @brief Main event handler function.
 * @param[in] eh Event_header for the if-chain to use to recognize which event triggered.
 * @return True if event is consumed, otherwise false.
 */
static bool event_handler(const struct event_header *eh)
{
	if (is_pwr_reboot_event(eh)) {
		reboot_scheduled = true;
		return false;
	}
	if (is_gnss_data(eh)) {
		struct gnss_data *ev = cast_gnss_data(eh);
		cached_gnss_mode = (gnss_mode_t)ev->gnss_data.lastfix.mode;

		/* Update that we received GNSS data regardless of validity. */
		update_cache_reg(GNSS_STRUCT);

		/* TODO, review pshustad, might block the event manager for 50 ms ? */
		if (k_sem_take(&cache_lock_sem, K_MSEC(50)) == 0) {
			if (ev->gnss_data.fix_ok && ev->gnss_data.has_lastfix) {
				cached_fix = ev->gnss_data.lastfix;
			}
			cached_ttff = ev->gnss_data.latest.ttff;
			cached_msss = ev->gnss_data.latest.msss;
			k_sem_give(&cache_lock_sem);
		}

		if (ev->gnss_data.fix_ok && ev->gnss_data.has_lastfix) {
			time_t gm_time = (time_t)ev->gnss_data.lastfix.unix_timestamp;
			struct tm *tm_time = gmtime(&gm_time);

			if (tm_time->tm_year < 2015) {
				LOG_DBG("Invalid gnss packet.");
				return false;
			}
			/* Update date_time library which storage uses for ANO data. */
			date_time_set(tm_time);
		}
		return false;
	}
	if (is_ble_cmd_event(eh)) {
		struct ble_cmd_event *ev = cast_ble_cmd_event(eh);
		while (k_msgq_put(&ble_cmd_msgq, ev, K_NO_WAIT) != 0) {
			k_msgq_purge(&ble_cmd_msgq);
		}
		return true;
	}
	if (is_cellular_proto_in_event(eh)) {
		struct cellular_proto_in_event *ev = cast_cellular_proto_in_event(eh);
		while (k_msgq_put(&lte_proto_msgq, ev, K_NO_WAIT) != 0) {
			k_msgq_purge(&lte_proto_msgq);
		}
		return true;
	}
	if (is_update_collar_mode(eh)) {
		Mode prev_collar_mode = current_state.collar_mode;

		struct update_collar_mode *ev = cast_update_collar_mode(eh);
		current_state.collar_mode = ev->collar_mode;
		update_cache_reg(COLLAR_MODE);

		if (has_initial_collar_states()) {
			if (prev_collar_mode != current_state.collar_mode) {
				/* Notify server by status message that collar mode has changed */
				int err = k_work_reschedule_for_queue(&message_q,
						&log_status_message_work, K_NO_WAIT);
				if (err < 0) {
					LOG_ERR("Failed to schedule log status work (%d)", err);
				}
			}
		} else {
			set_initial_collar_state_flag(COLLAR_MODE_FLAG);
		}
		return false;
	}
	if (is_update_collar_status(eh)) {
		CollarStatus prev_collar_status = current_state.collar_status;

		struct update_collar_status *ev = cast_update_collar_status(eh);
		current_state.collar_status = ev->collar_status;
		update_cache_reg(COLLAR_STATUS);

		if (has_initial_collar_states()) {
			if ((prev_collar_status != current_state.collar_status) &&
			    ((current_state.collar_status == CollarStatus_Stuck) ||
			    (prev_collar_status == CollarStatus_Stuck) ||
			    (current_state.collar_status == CollarStatus_OffAnimal) ||
			    (prev_collar_status == CollarStatus_OffAnimal))) {
				/* Notify server by status message that collar status has changed */
				int err = k_work_reschedule_for_queue(&message_q,
						&log_status_message_work, K_NO_WAIT);
				if (err < 0) {
					LOG_ERR("Failed to schedule log status work (%d)", err);
				}
			}
		} else {
			set_initial_collar_state_flag(COLLAR_STATUS_FLAG);
		}
		return false;
	}
	if (is_update_fence_status(eh)) {
		FenceStatus prev_fence_status = current_state.fence_status;

		struct update_fence_status *ev = cast_update_fence_status(eh);
		current_state.fence_status = ev->fence_status;
		update_cache_reg(FENCE_STATUS);

		if (has_initial_collar_states()) {
			if ((prev_fence_status != current_state.fence_status) &&
			    (((current_state.fence_status == FenceStatus_MaybeOutOfFence) ||
			    (prev_fence_status == FenceStatus_MaybeOutOfFence)) ||
			    (prev_fence_status == FenceStatus_Escaped) ||
			    ((current_state.fence_status == FenceStatus_FenceStatus_Normal) &&
			    (prev_fence_status == FenceStatus_NotStarted)) ||
			    ((current_state.fence_status == FenceStatus_TurnedOffByBLE) &&
			    (prev_fence_status == FenceStatus_TurnedOffByBLE)))) {
				/* Notify server by status message that fence status has changed */
				int err = k_work_reschedule_for_queue(&message_q,
						&log_status_message_work, K_NO_WAIT);
				if (err < 0) {
					LOG_ERR("Failed to schedule log status work (%d)", err);
				}
			}
		} else {
			set_initial_collar_state_flag(FENCE_STATUS_FLAG);
		}
		return false;
	}
	if (is_update_fence_version(eh)) {
		struct update_fence_version *ev = cast_update_fence_version(eh);
		current_state.fence_version = ev->fence_version;
		update_cache_reg(FENCE_VERSION);
		if (!m_transfer_boot_params) {
			/* Notify server as soon as the new fence is activated. */
			LOG_WRN("Schedule poll request: fence_version!");
			int err = k_work_reschedule_for_queue(&message_q, &modem_poll_work,
							      K_NO_WAIT);
			if (err < 0) {
				LOG_ERR("Error schedule poll request work (%d)", err);
			}
		}
		return false;
	}
	if (is_update_flash_erase(eh)) {
		current_state.flash_erase_count++;
		/** @todo Not written to storage. Should it? And also be added to
		 * update cache reg??
		 */
		/*update_cache_reg(FLASH_ERASE_COUNT);*/
		return false;
	}
	if (is_update_zap_count(eh)) {
		struct update_zap_count *ev = cast_update_zap_count(eh);
		current_state.zap_count = ev->count;
		update_cache_reg(ZAP_COUNT);
		return false;
	}
	if (is_amc_zapped_now_event(eh)) {
		struct amc_zapped_now_event *ev = cast_amc_zapped_now_event(eh);
		atomic_set(&cached_dist_zap, ev->fence_dist);

		int err = k_work_reschedule_for_queue(&message_q, &process_zap_work, K_NO_WAIT);
		if (err < 0) {
			LOG_ERR("Error scheduling zap work (%d)", err);
		}
		return false;
	}
	if (is_cellular_ack_event(eh)) {
		struct cellular_ack_event *ev = cast_cellular_ack_event(eh);
		if (ev->message_sent) {
			k_sem_give(&send_out_ack);
		} else {
			k_sem_reset(&send_out_ack);
		}
		return false;
	}

	if (is_animal_escape_event(eh)) {
		int err = k_work_reschedule_for_queue(&message_q, &process_escape_work, K_NO_WAIT);
		if (err < 0) {
			LOG_ERR("Error reschedule escape work (%d)", err);
			nf_app_error(ERR_MESSAGING, err, NULL, 0);
		}
		return false;
	}
	if (is_connection_state_event(eh)) {
		struct connection_state_event *ev = cast_connection_state_event(eh);
		if (ev->state) {
			k_sem_give(&connection_ready);
		} else {
			k_sem_reset(&connection_ready);
		}
		return false;
	}
	if (is_animal_warning_event(eh)) {
		struct animal_warning_event *ev = cast_animal_warning_event(eh);
		atomic_set(&cached_dist_warn, ev->fence_dist);

		int err = k_work_reschedule_for_queue(&message_q, &process_warning_work, K_NO_WAIT);
		if (err < 0) {
			LOG_ERR("Error reschedule warning work (%d)", err);
			nf_app_error(ERR_MESSAGING, err, NULL, 0);
		}
		return false;
	}
	if (is_pwr_status_event(eh)) {
		struct pwr_status_event *ev = cast_pwr_status_event(eh);
		if (ev->pwr_state != PWR_CHARGING) {
			/* We want battery voltage in deci volt */
			atomic_set(&cached_batt, (uint16_t)(ev->battery_mv / 10));
			set_initial_collar_state_flag(BATTERY_LVL_FLAG);
		} else {
			cached_chrg += ev->charging_ma;
		}
		return false;
	}
	if (is_send_poll_request_now(eh)) {
		LOG_DBG("Received a nudge on listening socket!");

		int err;
		err = k_work_reschedule_for_queue(&message_q, &modem_poll_work, K_NO_WAIT);
		if (err < 0) {
			LOG_ERR("Error starting modem poll worker in response to nudge on listening socket. (%d)", err);
			nf_app_error(ERR_MESSAGING, err, NULL, 0);
		}
		return false;
	}
	if (is_env_sensor_event(eh)) {
		struct env_sensor_event *ev = cast_env_sensor_event(eh);

		LOG_DBG("Event Temp: %.2f, humid %.3f, press %.3f", ev->temp, ev->humidity,
			ev->press);

		/* Multiply sensor values with scaling factor and cache */
		atomic_set(&cached_press, (uint32_t)(ev->press * 1000));
		atomic_set(&cached_hum, (uint32_t)(ev->humidity * 1000));
		atomic_set(&cached_temp, (uint32_t)(ev->temp * 100));
		return false;
	}
	if (is_warn_correction_start_event(eh)) {
		struct warn_correction_start_event *ev = cast_warn_correction_start_event(eh);
		atomic_set(&cached_dist_correction_start, ev->fence_dist);

		int err = k_work_reschedule_for_queue(&message_q,
						      &process_warning_correction_start_work,
						      K_NO_WAIT);
		if (err < 0) {
			LOG_ERR("Error reschedule warning correction start work (%d)", err);
			nf_app_error(ERR_MESSAGING, err, NULL, 0);
		}
		return false;
	}
	if (is_warn_correction_end_event(eh)) {
		struct warn_correction_end_event *ev = cast_warn_correction_end_event(eh);
		atomic_set(&cached_dist_correction_end, ev->fence_dist);

		int err;
		err = k_work_reschedule_for_queue(&message_q, &process_warning_correction_end_work,
						  K_NO_WAIT);
		if (err < 0) {
			LOG_ERR("Error reschedule warning correction end work (%d)", err);
			nf_app_error(ERR_MESSAGING, err, NULL, 0);
		}
		return false;
	}
	if (is_gsm_info_event(eh)) {
		LOG_WRN("GSM info received!");

		struct gsm_info_event *ev = cast_gsm_info_event(eh);
		rat = ev->gsm_info.rat;
		mnc = ev->gsm_info.mnc;
		rssi = ev->gsm_info.rssi;
		min_rssi = ev->gsm_info.min_rssi;
		max_rssi = ev->gsm_info.max_rssi;
		memcpy(ccid, ev->gsm_info.ccid, sizeof(ccid));

		LOG_WRN("RSSI, rat: %d, %d, %d, %d, %s", rssi, min_rssi, max_rssi, rat,
			log_strdup(ccid));
		update_cache_reg(GSM_INFO);
		return false;
	}
	if (is_block_fota_event(eh)) {
		struct block_fota_event *ev = cast_block_fota_event(eh);
		block_fota_request = ev->block_lte_fota;
		return false;
	}
	if (is_dfu_status_event(eh)) {
		struct dfu_status_event *fw_upgrade_event = cast_dfu_status_event(eh);

		if (fw_upgrade_event->dfu_status == DFU_STATUS_IDLE &&
		    fw_upgrade_event->dfu_error != 0) {
			fota_reset = true;

			/* DFU/FOTA is canceled, release the halt on log data trafic in the
			 * messaging tx thread */
			atomic_set(&m_halt_data_transfer, false);
		} else if (fw_upgrade_event->dfu_status != DFU_STATUS_IDLE) {
			/* DFU/FOTA has started or is in progress, halt log data trafic in the
			 * messaging tx thread */
			atomic_set(&m_halt_data_transfer, true);
		}
		return false;
	}
	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);
	return false;
}

bool validate_pasture()
{
	uint16_t crc = 0xFFFF;
	uint16_t pasture_value_16;
	uint32_t pasture_value_32;

	pasture_value_32 = pasture_temp.m.ul_fence_def_version;
	crc = nf_crc16_uint32(pasture_value_32, &crc);

	pasture_value_32 = pasture_temp.m.l_origin_lon;
	crc = nf_crc16_uint32(pasture_value_32, &crc);

	pasture_value_32 = pasture_temp.m.l_origin_lat;
	crc = nf_crc16_uint32(pasture_value_32, &crc);

	pasture_value_16 = pasture_temp.m.us_k_lon;
	crc = nf_crc16_uint16(pasture_value_16, &crc);

	pasture_value_16 = pasture_temp.m.us_k_lat;
	crc = nf_crc16_uint16(pasture_value_16, &crc);

	if (pasture_temp.m.ul_total_fences == 0) {
		/* Ignore CRC for No pasture */
		return crc == pasture_temp.m.us_pasture_crc;
	}

	for (uint8_t i = 0; i < pasture_temp.m.ul_total_fences; i++) {
		fence_t *target_fence = &pasture_temp.fences[i];
		for (uint8_t j = 0; j < target_fence->m.n_points; j++) {
			pasture_value_16 = target_fence->coordinates[j].s_x_dm;
			crc = nf_crc16_uint16(pasture_value_16, &crc);

			pasture_value_16 = target_fence->coordinates[j].s_y_dm;
			crc = nf_crc16_uint16(pasture_value_16, &crc);
		}
	}
	return crc == pasture_temp.m.us_pasture_crc;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, ble_cmd_event);
EVENT_SUBSCRIBE(MODULE, cellular_ack_event);
EVENT_SUBSCRIBE(MODULE, cellular_proto_in_event);
EVENT_SUBSCRIBE(MODULE, update_collar_mode);
EVENT_SUBSCRIBE(MODULE, update_collar_status);
EVENT_SUBSCRIBE(MODULE, update_fence_status);
EVENT_SUBSCRIBE(MODULE, update_fence_version);
EVENT_SUBSCRIBE(MODULE, update_flash_erase);
EVENT_SUBSCRIBE(MODULE, update_zap_count);
EVENT_SUBSCRIBE(MODULE, animal_warning_event);
EVENT_SUBSCRIBE(MODULE, animal_escape_event);
EVENT_SUBSCRIBE(MODULE, amc_zapped_now_event);
EVENT_SUBSCRIBE(MODULE, connection_state_event);
EVENT_SUBSCRIBE(MODULE, pwr_status_event);
EVENT_SUBSCRIBE(MODULE, env_sensor_event);
/** @todo add battery, histogram, gnss and modem event */
EVENT_SUBSCRIBE(MODULE, gnss_data);
EVENT_SUBSCRIBE(MODULE, send_poll_request_now);
EVENT_SUBSCRIBE(MODULE, warn_correction_start_event);
EVENT_SUBSCRIBE(MODULE, warn_correction_end_event);
EVENT_SUBSCRIBE(MODULE, gsm_info_event);
EVENT_SUBSCRIBE(MODULE, dfu_status_event);
EVENT_SUBSCRIBE(MODULE, block_fota_event);

/**
 * @brief Process commands recieved on the bluetooth interface, and performs the appropriate
 * actions accordingly.
 */
static inline void process_ble_cmd_event(void)
{
	struct ble_cmd_event ev;

	int err = k_msgq_get(&ble_cmd_msgq, &ev, K_NO_WAIT);
	if (err) {
		LOG_ERR("Error getting ble_cmd_event (%d)", err);
		nf_app_error(ERR_MESSAGING, err, NULL, 0);
		return;
	}
	enum command_char ble_command = ev.cmd;

	switch (ble_command) {
		case CMD_TURN_OFF_FENCE: {
			/* Wait for final AMC integration. Should simply issue an event. */
			struct turn_off_fence_event *ev = new_turn_off_fence_event();
			EVENT_SUBMIT(ev);
			break;
		}
		case CMD_REBOOT_AVR_MCU: {
			struct pwr_reboot_event *r_ev = new_pwr_reboot_event();
			r_ev->reason = REBOOT_BLE_RESET;
			EVENT_SUBMIT(r_ev);
			break;
		}
		case CMD_PLAY_SOUND: {
			struct sound_event *s_ev = new_sound_event();
			s_ev->type = SND_FIND_ME;
			EVENT_SUBMIT(s_ev);
			break;
		}
		case CMD_DOWNLOAD_FENCE: {
			/* Downloads fence from bluetooth? */
			break;
		}
		case CMD_FW_INSTALL: {
			/* Deprecated as we have ble_dfu.c instead? */
			break;
		}
		default: {
			break;
		}
	}
}

/**
 * @brief Process Protobuf messages recieved on the LTE interface, and performs the appropriate
 * actions accordingly.
 */
static void process_lte_proto_event(void)
{
	struct cellular_proto_in_event ev;

	int err = k_msgq_get(&lte_proto_msgq, &ev, K_NO_WAIT);
	if (err) {
		LOG_ERR("Error getting lte_proto_event (%d)", err);
		nf_app_error(ERR_MESSAGING, err, NULL, 0);
		return;
	}

	NofenceMessage proto;
	err = collar_protocol_decode(ev.buf + 2, ev.len - 2, &proto);

	struct messaging_ack_event *ack = new_messaging_ack_event();
	EVENT_SUBMIT(ack);
	if (err) {
		LOG_ERR("Error decoding protobuf message (%d)", err);
		nf_app_error(ERR_MESSAGING, err, NULL, 0);
		return;
	}

	/* Process poll response if data transfer is not otherwie halted for some reason, e.g. a
	 * pending or ongoing FOTA */
	if (atomic_get(&m_halt_data_transfer) == 0)
	{
		if (proto.which_m == NofenceMessage_poll_message_resp_tag) {
			LOG_INF("Process poll reponse");
			process_poll_response(&proto);
			return;
		} else if (proto.which_m == NofenceMessage_fence_definition_resp_tag) {
			uint8_t received_frame = process_fence_msg(&proto.m.fence_definition_resp);
			fence_download(received_frame);
			return;
		} else if (proto.which_m == NofenceMessage_ubx_ano_reply_tag) {
			uint16_t new_ano_frame = process_ano_msg(&proto.m.ubx_ano_reply);
			ano_download(proto.m.ubx_ano_reply.usAnoId, new_ano_frame);
			return;
		} else {
			return;
		}
	}
}

/**
 * @brief Messaging module Rx thread.
 */
void messaging_rx_thread_fn()
{
	while (true) {
		int rc = k_poll(msgq_events, NUM_MSGQ_EVENTS, K_FOREVER);
		if (rc == 0) {
			while (k_msgq_num_used_get(&ble_cmd_msgq) > 0) {
				process_ble_cmd_event();
			}
			while (k_msgq_num_used_get(&lte_proto_msgq) > 0) {
				process_lte_proto_event();
			}
		}
		/* Set all the events to not ready again. */
		for (int i = 0; i < NUM_MSGQ_EVENTS; i++) {
			msgq_events[i].state = K_POLL_STATE_NOT_READY;
		}
	}
}

int messaging_module_init(void)
{
	LOG_INF("Initializing messaging module.");
	int err = stg_config_u32_read(STG_U32_UID, &serial_id);
	if (err != 0) {
		LOG_ERR("Failed to read serial number from storage! (%d)", err);
		nf_app_error(ERR_MESSAGING, err, NULL, 0);
		serial_id = 1; /* Fallback if read from storage fails */
	}

	/* Startup the modem to get the gsm_info ready before the first poll request.*/
	struct check_connection *ev = new_check_connection();
	EVENT_SUBMIT(ev);

	k_work_queue_init(&message_q);
	k_work_queue_start(&message_q, messaging_send_thread,
			   K_THREAD_STACK_SIZEOF(messaging_send_thread),
			   K_PRIO_COOP(CONFIG_MESSAGING_SEND_THREAD_PRIORITY),
			   NULL);

	k_work_init_delayable(&modem_poll_work, modem_poll_work_fn);
	k_work_init_delayable(&log_periodic_work, log_data_periodic_fn);
	k_work_init_delayable(&log_send_work, log_send_work_fn);
	k_work_init_delayable(&log_status_message_work, log_status_message_fn);
	k_work_init_delayable(&process_escape_work, log_animal_escaped_work_fn);
	k_work_init_delayable(&process_zap_work, log_zap_message_work_fn);
	k_work_init_delayable(&process_warning_work, log_warning_work_fn);
	k_work_init_delayable(&process_warning_correction_start_work, log_correction_start_work_fn);
	k_work_init_delayable(&process_warning_correction_end_work, log_correction_end_work_fn);
	k_work_init_delayable(&data_request_work, data_request_work_fn);
	k_work_init_delayable(&m_fence_update_req.work, fence_update_req_fn);

	memset(&pasture_temp, 0, sizeof(pasture_t));
	cached_fences_counter = 0;
	pasture_temp.m.us_pasture_crc = EMPTY_FENCE_CRC;

	/** @todo Should add semaphore and only start these queues when
	 *  we get connection to network with modem.
	 */

	err = k_work_schedule_for_queue(&message_q, &data_request_work, K_NO_WAIT);
	if (err < 0) {
		return err;
	}
	err = k_work_schedule_for_queue(&message_q, &modem_poll_work, K_SECONDS(2));
	if (err < 0) {
		return err;
	}
	err = k_work_schedule_for_queue(&message_q, &log_periodic_work,
					K_MINUTES(atomic_get(&log_period_minutes)));
	if (err < 0) {
		return err;
	}
	return 0;
}

/**
 * @brief Builds a poll request with the latest data.
 * @param poll_req The poll request message.
 */
void build_poll_request(NofenceMessage *poll_req)
{
	int err;

	proto_InitHeader(poll_req);
	poll_req->which_m = NofenceMessage_poll_message_req_tag;
	proto_get_last_known_date_pos(&cached_fix, &poll_req->m.poll_message_req.datePos);
	poll_req->m.poll_message_req.has_datePos = proto_has_last_known_date_pos(&cached_fix);
	poll_req->m.poll_message_req.eMode = current_state.collar_mode;
	poll_req->m.poll_message_req.usZapCount = current_state.zap_count;
	poll_req->m.poll_message_req.eCollarStatus = current_state.collar_status;
	poll_req->m.poll_message_req.eFenceStatus = current_state.fence_status;
	poll_req->m.poll_message_req.ulFenceDefVersion = current_state.fence_version;
	poll_req->m.poll_message_req.usBatteryVoltage = (uint16_t)atomic_get(&cached_batt);
	poll_req->m.poll_message_req.has_xGsmInfo = true;

	_GSM_INFO p_gsm_info;
	p_gsm_info.ucRAT = (uint8_t)rat;
	sprintf(p_gsm_info.xMMC_MNC, "%d", mnc);

	poll_req->m.poll_message_req.xGsmInfo = p_gsm_info;
	poll_req->m.poll_message_req.has_xGsmInfo = true;

	if (current_state.flash_erase_count) {
		// m_flash_erase_count is reset when we receive a poll reply
		poll_req->m.poll_message_req.has_usFlashEraseCount = true;
		poll_req->m.poll_message_req.usFlashEraseCount = current_state.flash_erase_count;
	}
	if (m_confirm_acc_limits || m_transfer_boot_params) {
		/** @warning Assumes all the activity values are given with the
		 *  m_confirm_acc_limits flag set in poll response from server.
		 */
		poll_req->m.poll_message_req.has_usAccSigmaSleepLimit = true;
		poll_req->m.poll_message_req.has_usAccSigmaNoActivityLimit = true;
		poll_req->m.poll_message_req.has_usOffAnimalTimeLimitSec = true;

		err = stg_config_u16_read(STG_U16_ACC_SIGMA_SLEEP_LIMIT, 
					  &poll_req->m.poll_message_req.usAccSigmaSleepLimit);
		if (err != 0) {
			LOG_ERR("Failed to read STG_U16_ACC_SIGMA_SLEEP_LIMIT (%d)", err);
			nf_app_error(ERR_MESSAGING, err, NULL, 0);
		}

		err = stg_config_u16_read(STG_U16_ACC_SIGMA_NOACTIVITY_LIMIT, 
					  &poll_req->m.poll_message_req.usAccSigmaNoActivityLimit);
		if (err != 0) {
			LOG_ERR("Failed to read STG_U16_ACC_SIGMA_NOACTIVITY_LIMIT (%d)", err);
			nf_app_error(ERR_MESSAGING, err, NULL, 0);
		}

		err = stg_config_u16_read(STG_U16_OFF_ANIMAL_TIME_LIMIT_SEC, 
					  &poll_req->m.poll_message_req.usOffAnimalTimeLimitSec);
		if (err != 0) {
			LOG_ERR("Failed to read STG_U16_OFF_ANIMAL_TIME_LIMIT_SEC (%d)", err);
			nf_app_error(ERR_MESSAGING, err, NULL, 0);
		}
	}
	if (m_confirm_ble_key || m_transfer_boot_params) {
		poll_req->m.poll_message_req.has_rgubcBleKey = true;
		poll_req->m.poll_message_req.rgubcBleKey.size = STG_CONFIG_BLE_SEC_KEY_LEN;
		uint8_t key_length = 0;
		err = stg_config_blob_read(STG_BLOB_BLE_KEY, 
					   poll_req->m.poll_message_req.rgubcBleKey.bytes,
					   &key_length);
		if (err != 0) {
			LOG_ERR("Failed to read ble_sec_key (%d)", err);
			nf_app_error(ERR_MESSAGING, err, NULL, 0);
		}
	}
	/* TODO pshustad, fill GNSSS parameters for MIA M10 */
	poll_req->m.poll_message_req.has_usGnssOnFixAgeSec = 
		(cached_gnss_mode == GNSSMODE_NOMODE) ? false : true;
	uint32_t timeSinceFixSec = (cached_msss - cached_fix.msss)/1000;
	if (timeSinceFixSec > UINT16_MAX) {
		timeSinceFixSec = UINT16_MAX;
	}
	poll_req->m.poll_message_req.usGnssOnFixAgeSec = timeSinceFixSec;

	poll_req->m.poll_message_req.has_usGnssTTFFSec = 
		(cached_gnss_mode == GNSSMODE_NOMODE) ? false : true;
	uint32_t timeSinceFirstFixSec = cached_ttff/1000;
	if (timeSinceFirstFixSec > UINT16_MAX) {
		timeSinceFirstFixSec = UINT16_MAX;
	}
	poll_req->m.poll_message_req.usGnssTTFFSec = timeSinceFirstFixSec;

	if (m_transfer_boot_params) {
		poll_req->m.poll_message_req.has_versionInfo = true;
		poll_req->m.poll_message_req.versionInfo.has_ulApplicationVersion = true;
		poll_req->m.poll_message_req.versionInfo.ulApplicationVersion =
			NF_X25_VERSION_NUMBER;
		if (ccid[0] != '\0') {
			poll_req->m.poll_message_req.has_xSimCardId = true;
			memcpy(poll_req->m.poll_message_req.xSimCardId, ccid, 
			       sizeof(poll_req->m.poll_message_req.xSimCardId) - 1);
		}

		/* TODO pshustad, clean up and re-enable the commented code below */
		//		uint16_t xbootVersion;
		//		if (xboot_get_version(&xbootVersion) == XB_SUCCESS) {
		//			poll_req.m.poll_message_req.versionInfo
		//				.usATmegaBootloaderVersion =
		//				xbootVersion;
		//			poll_req.m.poll_message_req.versionInfo
		//				.has_usATmegaBootloaderVersion = true;
		//		}

		poll_req->m.poll_message_req.has_versionInfoHW = true;

		uint8_t pcb_rf_version = 0;
		stg_config_u8_read(STG_U8_HW_VERSION, &pcb_rf_version);
		poll_req->m.poll_message_req.versionInfoHW.ucPCB_RF_Version = pcb_rf_version;

		uint16_t pcb_product_type = 0;
		stg_config_u16_read(STG_U16_PRODUCT_TYPE, &pcb_product_type);
		poll_req->m.poll_message_req.versionInfoHW.usPCB_Product_Type = pcb_product_type;

		uint8_t bom_mec_rev = 0;
		stg_config_u8_read(STG_U8_BOM_MEC_REV, &bom_mec_rev);
		poll_req->m.poll_message_req.versionInfoBOM.ucBom_mec_rev = bom_mec_rev;
		poll_req->m.poll_message_req.has_versionInfoBOM = true;

		uint8_t bom_pcb_rev = 0;
		stg_config_u8_read(STG_U8_BOM_PCB_REV, &bom_pcb_rev);
		poll_req->m.poll_message_req.versionInfoBOM.ucBom_pcb_rev = bom_pcb_rev;

		uint8_t ems_provider = 0;
		stg_config_u8_read(STG_U8_EMS_PROVIDER, &ems_provider);
		poll_req->m.poll_message_req.versionInfoBOM.ucEms_provider = ems_provider;

		uint8_t record_rev = 0;
		stg_config_u8_read(STG_U8_PRODUCT_RECORD_REV, &record_rev);
		poll_req->m.poll_message_req.versionInfoBOM.ucProduct_record_rev = record_rev;

		uint8_t reboot_reason;
		pwr_module_reboot_reason(&reboot_reason);
		poll_req->m.poll_message_req.has_ucMCUSR = true;
		poll_req->m.poll_message_req.ucMCUSR = reboot_reason;

		/** @todo Add information of SIM card */
#if 0
		poll_req.m.poll_message_req.has_xSimCardId = true;
		memcpy(poll_req.m.poll_message_req.xSimCardId, BGS_SCID(),
			sizeof(poll_req.m.poll_message_req.xSimCardId));
		poll_req.m.poll_message_req.versionInfo.has_ulATmegaVersion =
			true;
		poll_req.m.poll_message_req.versionInfo.ulATmegaVersion =
			NF_X25_VERSION_NUMBER;
#endif
	}
}

/**
 * @brief Handler for a new fence definition download from the server.
 * @param received_frame A received frame of the new fence defintion.
 */
void fence_download(uint8_t received_frame)
{
	if (received_frame == DOWNLOAD_COMPLETE) {
		/* Notify AMC that a new fence is available. */
		struct new_fence_available *fence_ready = new_new_fence_available();
		fence_ready->new_fence_version = m_fence_update_req.version;
		EVENT_SUBMIT(fence_ready);

		LOG_INF("Fence ver %d download complete and notified AMC.",
			m_fence_update_req.version);
	} else if (received_frame == m_fence_update_req.request_frame) {
		m_fence_update_req.request_frame++;

		LOG_INF("Requesting frame %d of new fence: %d", m_fence_update_req.request_frame,
			m_fence_update_req.version);

		/* Submit a fence frame message request */
		int err = k_work_reschedule_for_queue(&message_q, &m_fence_update_req.work,
						      K_NO_WAIT);
		if (err < 0) {
			LOG_ERR("Failed to reschedule work");
		}
	} else {
		/* Received incorrect fence frame number, cancel download */
		m_fence_update_req.version = current_state.fence_version;
		m_fence_update_req.request_frame = 0;
	}
}

/**
 * @brief Builds and sends a ANO data request to the server.
 *
 * @param ano_id The identifier of the ANO request.
 * @param ano_start The start identifier for the ANO request.
 * @return Returns 0 if successfull, otherwise a negative error code.
 */
int8_t request_ano_frame(uint16_t ano_id, uint16_t ano_start)
{
	NofenceMessage ano_req;
	proto_InitHeader(&ano_req); /* fill up message header. */
	ano_req.which_m = NofenceMessage_ubx_ano_req_tag;
	ano_req.m.ubx_ano_req.usAnoId = ano_id;
	ano_req.m.ubx_ano_req.usStartAno = ano_start;
	int ret = encode_and_send_message(&ano_req);
	if (ret) {
		LOG_ERR("Failed to send request for ano %d (%d)", ano_start, ret);
		nf_app_error(ERR_MESSAGING, ret, NULL, 0);
		return -1;
	}
	return 0;
}

/**
 * @brief Handler for ANO data download.
 * @param ano_id The identifier of the ANO request.
 * @param new_ano_frame A frame of the ANO data download.
 */
void ano_download(uint16_t ano_id, uint16_t new_ano_frame)
{
	if (first_ano_frame) {
		new_ano_in_progress = ano_id;
		expected_ano_frame = 0;
		first_ano_frame = false;
	} else if (new_ano_frame == 0 && !first_ano_frame) {
		expected_ano_frame = 0;
		new_ano_in_progress = 0;
		return;
	}
	if (new_ano_frame >= 0) {
		if (new_ano_frame == DOWNLOAD_COMPLETE) {
			LOG_INF("ANO %d download complete.", new_ano_in_progress);
			return;
		}

		expected_ano_frame += new_ano_frame;
		/* TODO: handle failure to send request!*/
		int ret = request_ano_frame(ano_id, expected_ano_frame);

		LOG_INF("Requesting frame %d of new ano: %d.", expected_ano_frame,
			new_ano_in_progress);

		if (ret != 0) {
			/* TODO: reset ANO state and retry later.*/
			expected_ano_frame = 0;
			new_ano_in_progress = 0;
		}
		return;
	}
}

/**
 * @brief Initialize a Nofence protobuf message (See NofenceMessage).
 * @param msg The initialized Nofence protobuf message.
 */
void proto_InitHeader(NofenceMessage *msg)
{
	memset(msg, 0, sizeof(NofenceMessage));
	msg->header.ulId = serial_id;
	msg->header.ulVersion = NF_X25_VERSION_NUMBER;
	msg->header.has_ulVersion = true;
	static int64_t curr_time = 0;
	if (!date_time_now(&curr_time)) {
		/* Convert to seconds since 1.1.1970 */
		msg->header.ulUnixTimestamp = (uint32_t)(curr_time / 1000);
	} else {
		/** @todo: Add uptime instead? */
		msg->header.ulUnixTimestamp = 0;
	}
}

/**
 * @brief Sends a binary encoded message to the server through cellular controller.
 * @param data Binary message. @note Assumes 2 first bytes are empty.
 * @param len Length of the binary data including the 2 start bytes.
 * @return Returns 0 if successfull, otherwise negative error code.
 */
int send_binary_message(uint8_t *data, size_t len)
{
	/* We can only send 1 message at a time, use mutex. */
	k_mutex_lock(&send_binary_mutex, K_NO_WAIT);
	if (send_binary_mutex.lock_count == 1) {
		struct check_connection *ev = new_check_connection();
		EVENT_SUBMIT(ev);

		int ret = k_sem_take(&connection_ready, K_FOREVER);
		if (ret != 0) {
			LOG_ERR("Connection not ready, can't send message now! (%d)", ret);
			nf_app_error(ERR_MESSAGING, ret, NULL, 0);
			k_mutex_unlock(&send_binary_mutex);
			return ret;
		}
		uint16_t byteswap_size = BYTESWAP16(len - 2);
		memcpy(&data[0], &byteswap_size, 2);

		struct messaging_proto_out_event *msg2send = new_messaging_proto_out_event();
		msg2send->buf = data;
		msg2send->len = len;
		EVENT_SUBMIT(msg2send);

		ret = k_sem_take(&send_out_ack, K_FOREVER);
		if (ret != 0) {
			LOG_WRN("Message not sent!");
			nf_app_error(ERR_MESSAGING, ret, NULL, 0);
			k_mutex_unlock(&send_binary_mutex);
			return ret;
		}
		k_mutex_unlock(&send_binary_mutex);
		return 0;
	} else {
		k_mutex_unlock(&send_binary_mutex);
		return -EBUSY;
	}
}

/**
 * @brief Encodes a message according to the Nofence protofuf protocol and sends the binary
 * message to the server through the cellular controller.
 * @param msg_proto The message to encode and send.
 * @return Returns 0 if successfull, otherwise a negative error code.
 */
int encode_and_send_message(NofenceMessage *msg_proto)
{
	uint8_t encoded_msg[NofenceMessage_size];
	memset(encoded_msg, 0, sizeof(encoded_msg));
	size_t encoded_size = 0;
	size_t header_size = 2;

	LOG_INF("Start message encoding, tag: %u, version: %u",
		msg_proto->which_m, msg_proto->header.ulVersion);
	int ret = collar_protocol_encode(msg_proto, &encoded_msg[2],
					 NofenceMessage_size, &encoded_size);
	if (ret) {
		LOG_ERR("Error encoding nofence message (%d)", ret);
		nf_app_error(ERR_MESSAGING, ret, NULL, 0);
		return ret;
	}
	return send_binary_message(encoded_msg, encoded_size + header_size);
}

/**
 * @brief Encodes a message according to the Nofence protofuf protocol and stores the message to
 * external storage.
 * @param msg_proto The message to encode and store.
 * @return Returns 0 if successfull, otherwise a negative error code.
 */
int encode_and_store_message(NofenceMessage *msg_proto)
{
	int ret;
	size_t encoded_size = 0;
	size_t header_size = 2;
	uint8_t encoded_msg[NofenceMessage_size];
	memset(encoded_msg, 0, sizeof(encoded_msg));

	LOG_INF("Start message encoding, tag: %u, version: %u",
		msg_proto->which_m, msg_proto->header.ulVersion);
	ret = collar_protocol_encode(msg_proto, &encoded_msg[2],
					 NofenceMessage_size, &encoded_size);
	if (ret) {
		LOG_ERR("Error encoding nofence message (%d)", ret);
		nf_app_error(ERR_MESSAGING, ret, NULL, 0);
		return ret;
	}
	uint16_t total_size = encoded_size + header_size;

	/* Store the length of the message in the two first bytes */
	memcpy(&encoded_msg[0], &total_size, 2);

	ret = stg_write_log_data(encoded_msg, (size_t)total_size);
	if (ret != 0) {
		LOG_ERR("Failed to store message to flash!");
		return ret;
	}
	return ret;
}

/**
 * @brief Process an incoming poll response and performs the appropriate actions accordingly.
 * @param proto The incoming protobuf message to process.
 */
void process_poll_response(NofenceMessage *proto)
{
	int err;

	/* When we receive a poll reply, we don't want to transfer boot params */
	m_transfer_boot_params = false;
	PollMessageResponse *pResp = &proto->m.poll_message_resp;
	if (pResp->has_xServerIp && strlen(pResp->xServerIp) > 0) {
		struct messaging_host_address_event *host_add_event =
			new_messaging_host_address_event();
		strncpy(host_add_event->address, pResp->xServerIp, sizeof(pResp->xServerIp));
		EVENT_SUBMIT(host_add_event); 
		/*cellular controller writes it to ext flash if it is different from the 
		 * previously stored address.*/
	}
	if (pResp->has_bEraseFlash && pResp->bEraseFlash) {
		struct request_flash_erase_event *flash_erase_event =
			new_request_flash_erase_event();
		flash_erase_event->magic = STORAGE_ERASE_MAGIC;
		EVENT_SUBMIT(flash_erase_event);
	}
	// If we are asked to, reboot
	if (pResp->has_bReboot && pResp->bReboot) {
		struct pwr_reboot_event *r_ev = new_pwr_reboot_event();
		EVENT_SUBMIT(r_ev);
	}
	/* TODO: set activation mode to (pResp->eActivationMode); */

	if (pResp->has_bUseUbloxAno) {
		/* TODO: publish enable ANO event to GPS controller */
	}
	if (pResp->has_bUseServerTime && pResp->bUseServerTime) {
		LOG_INF("Set date and time from server");
		time_t gm_time = (time_t)proto->header.ulUnixTimestamp;
		struct tm *tm_time = gmtime(&gm_time);
		/* Update date_time library which storage uses for ANO data. */
		err = date_time_set(tm_time);
		if (err) {
			LOG_ERR("Error updating time from server (%d)", err);
			nf_app_error(ERR_MESSAGING, err, NULL, 0);
		} else {
			/** @note This prints UTC. */
			LOG_INF("Set timestamp to date_time library from modem: %s",
				log_strdup(asctime(tm_time)));
			atomic_set(&server_timestamp_sec,
				   (atomic_val_t)(int32_t)(k_uptime_get_32() / 1000));
		}
	}
	if (pResp->has_usPollConnectIntervalSec) {
		/* Update poll request interval if not equal to interval requested by server */
		if (atomic_get(&poll_period_seconds) != pResp->usPollConnectIntervalSec) {
			atomic_set(&poll_period_seconds, pResp->usPollConnectIntervalSec);

			err = k_work_reschedule_for_queue(&message_q, &modem_poll_work,
						    K_SECONDS(atomic_get(&poll_period_seconds)));
			if (err < 0) {
				LOG_ERR("Failed to schedule work");
			}

			LOG_INF("Poll period of %d seconds will be used",
				atomic_get(&poll_period_seconds));
		}
	}
	m_confirm_acc_limits = false;
	if (pResp->has_usAccSigmaSleepLimit) {
		m_confirm_acc_limits = true;
		err = stg_config_u16_write(STG_U16_ACC_SIGMA_SLEEP_LIMIT,
					   pResp->usAccSigmaSleepLimit);
		if (err != 0) {
			LOG_ERR("Error updating sleep sigma to ext flash (%d)", err);
			nf_app_error(ERR_MESSAGING, err, NULL, 0);
		}

		struct acc_sigma_event *sigma_ev = new_acc_sigma_event();
		sigma_ev->type = SLEEP_SIGMA;
		sigma_ev->param.sleep_sigma_value = pResp->usAccSigmaSleepLimit;
		EVENT_SUBMIT(sigma_ev);
	}
	if (pResp->has_usAccSigmaNoActivityLimit) {
		m_confirm_acc_limits = true;
		err = stg_config_u16_write(STG_U16_ACC_SIGMA_NOACTIVITY_LIMIT,
					   pResp->usAccSigmaNoActivityLimit);
		if (err != 0) {
			LOG_ERR("Error updating no activity sigma to ext flash (%d)", err);
			nf_app_error(ERR_MESSAGING, err, NULL, 0);
		}

		struct acc_sigma_event *sigma_ev = new_acc_sigma_event();
		sigma_ev->type = NO_ACTIVITY_SIGMA;
		sigma_ev->param.no_activity_sigma = pResp->usAccSigmaNoActivityLimit;
		EVENT_SUBMIT(sigma_ev);
	}
	if (pResp->has_usOffAnimalTimeLimitSec) {
		m_confirm_acc_limits = true;
		err = stg_config_u16_write(STG_U16_OFF_ANIMAL_TIME_LIMIT_SEC,
					   pResp->usOffAnimalTimeLimitSec);
		if (err != 0) {
			LOG_ERR("Error updating off animal sigma to ext flash (%d)", err);
			nf_app_error(ERR_MESSAGING, err, NULL, 0);
		}

		struct acc_sigma_event *sigma_ev = new_acc_sigma_event();
		sigma_ev->type = OFF_ANIMAL_SIGMA;
		sigma_ev->param.off_animal_value = pResp->usOffAnimalTimeLimitSec;
		EVENT_SUBMIT(sigma_ev);
	}

	m_confirm_ble_key = false;
	if (pResp->has_rgubcBleKey) {
		m_confirm_ble_key = true;
		LOG_INF("Received a ble_sec_key of size %d", pResp->rgubcBleKey.size);

		uint8_t current_ble_sec_key[STG_CONFIG_BLE_SEC_KEY_LEN];
		uint8_t key_length = 0;
		stg_config_blob_read(STG_BLOB_BLE_KEY, current_ble_sec_key, &key_length);
		int ret = memcmp(pResp->rgubcBleKey.bytes, current_ble_sec_key,
				 pResp->rgubcBleKey.size);
		if (ret != 0) {
			LOG_INF("New ble sec key is different. Will update ext flash");
			ret = stg_config_blob_write(STG_BLOB_BLE_KEY, pResp->rgubcBleKey.bytes,
						    pResp->rgubcBleKey.size);
			if (ret < 0) {
				LOG_ERR("Failed to write ble sec key to ext flash (%d)", ret);
				nf_app_error(ERR_MESSAGING, ret, NULL, 0);
			}
		}
	}
	if (pResp->ulFenceDefVersion != current_state.fence_version) {
		LOG_INF("Requesting frame 0 for fence version %i.", pResp->ulFenceDefVersion);

		/* Submit a fence frame message request */
		m_fence_update_req.version = pResp->ulFenceDefVersion;
		m_fence_update_req.request_frame = 0;
		err = k_work_reschedule_for_queue(&message_q, &m_fence_update_req.work, K_NO_WAIT);
		if (err < 0) {
			LOG_ERR("Failed to schedule work");
		}
		return;
	}

	if (pResp->has_versionInfo) {
		process_upgrade_request(&pResp->versionInfo);
	}
	return;
}

/**
 * @brief Process an incoming firmware upgrade request, and starts the firmware download if a new
 * version is available on the server.
 * @param fw_ver_from_server The firmware version available on the server.
 */
void process_upgrade_request(VersionInfoFW *fw_ver_from_server)
{
	if (fw_ver_from_server->has_ulApplicationVersion &&
	    fw_ver_from_server->ulApplicationVersion != NF_X25_VERSION_NUMBER && 
	    block_fota_request == false) {
		LOG_INF("Received new app version from server %i",
			fw_ver_from_server->ulApplicationVersion);
		if (!reboot_scheduled) {
			struct start_fota_event *ev = new_start_fota_event();
			ev->override_default_host = false;
			ev->reset_download_client = fota_reset;
			ev->version = fw_ver_from_server->ulApplicationVersion;
			EVENT_SUBMIT(ev);
			fota_reset = false;
		}
	}
}

/** @brief Process a fence frame and stores it into the cached pasture so we can validate if its
 * valid when we're done to further store on external flash.
 * @param fenceResp fence frame received from server.
 * @returns 0 = requests the first frame again, something happened.
 * @returns 1-254 = frame number processed. This is used to know which frame to request next.
 * @returns Do we need negative error code if error, I.e pasture is not valid, or just return 0
 * and retry forever?
 * @returns 255 if download is complete.
 */
uint8_t process_fence_msg(FenceDefinitionResponse *fenceResp)
{
	if (fenceResp == NULL) {
		return 0;
	}
	uint8_t frame = fenceResp->ucFrameNumber;

	int err = 0;
	if (m_fence_update_req.version != fenceResp->ulFenceDefVersion) {
		/* Something went wrong, restart fence request. */
		return 0;
	}

	if (frame == 0) {
		memset(&pasture_temp, 0, sizeof(pasture_t));
		cached_fences_counter = 0;
		pasture_temp.m.us_pasture_crc = EMPTY_FENCE_CRC;
	}

	if (fenceResp->which_m == FenceDefinitionResponse_xHeader_tag) {
		if (frame != 0) {
			/* We always expect the header to be the first frame. */
			LOG_ERR("Unexpected frame count for pasture header. (%d)", -EIO);
			nf_app_error(ERR_MESSAGING, -EIO, NULL, 0);

			return 0;
		}

		/* Pasture header. */
		if (fenceResp->m.xHeader.has_bKeepMode) {
			pasture_temp.m.has_keep_mode = true;
			pasture_temp.m.keep_mode = fenceResp->m.xHeader.bKeepMode;

			/* Write to ext flash storage. */
			err = stg_config_u8_write(STG_U8_KEEP_MODE, 
					(uint8_t)fenceResp->m.xHeader.bKeepMode);
			if (err != 0) {
				/* We always expect the header to be the first frame. */
				LOG_ERR("Error writing keep mode to storage. (%d)", err);
				nf_app_error(ERR_MESSAGING, err, NULL, 0);
			}
		}

		if (fenceResp->has_usFenceCRC) {
			pasture_temp.m.has_us_pasture_crc = true;
			pasture_temp.m.us_pasture_crc = fenceResp->usFenceCRC;
		}
		pasture_temp.m.l_origin_lat = fenceResp->m.xHeader.lOriginLat;
		pasture_temp.m.l_origin_lon = fenceResp->m.xHeader.lOriginLon;
		pasture_temp.m.us_k_lat = fenceResp->m.xHeader.usK_LAT;
		pasture_temp.m.us_k_lon = fenceResp->m.xHeader.usK_LON;
		pasture_temp.m.ul_fence_def_version = fenceResp->ulFenceDefVersion;
		pasture_temp.m.ul_total_fences = fenceResp->m.xHeader.ulTotalFences;

	} else if (FenceDefinitionResponse_xFence_tag) {
		/* Fence frame info to store into pasture's fence array. */
		fence_t *loc = &pasture_temp.fences[cached_fences_counter];

		/* Fence header. */
		loc->m.n_points = fenceResp->m.xFence.rgulPoints_count;
		loc->m.us_id = fenceResp->m.xFence.usId;
		loc->m.e_fence_type = fenceResp->m.xFence.eFenceType;
		loc->m.fence_no = fenceResp->m.xFence.fenceNo;

		/* Fence coordinates. */
		memcpy(loc->coordinates, fenceResp->m.xFence.rgulPoints,
		       loc->m.n_points * sizeof(fence_coordinate_t));

		/* Increment number of fences stored in pasture. */
		cached_fences_counter++;
	}

	LOG_INF("Cached fence frame %i successfully.", frame);
	if (frame == fenceResp->ucTotalFrames - 1) {
		/* Validate pasture. */
		if (cached_fences_counter != pasture_temp.m.ul_total_fences) {
			LOG_ERR("Cached %i frames, but expected %i.", cached_fences_counter,
				pasture_temp.m.ul_total_fences);
						nf_app_error(ERR_MESSAGING, -EIO, NULL, 0);

			return 0;
		}

		if (pasture_temp.m.ul_total_fences == 0) {
			/* No pasture*/
			err = stg_write_pasture_data((uint8_t *)&pasture_temp,
						     sizeof(pasture_temp));
			if (err) {
				return err;
			}
			return DOWNLOAD_COMPLETE;
		}

		if (!validate_pasture()) {
			LOG_ERR("CRC was not correct for new pasture. (%d)", -EIO);
			nf_app_error(ERR_MESSAGING, -EIO, NULL, 0);
			return 0;
		}

		LOG_INF("Validated CRC for pasture and will write it to flash.");
		err = stg_write_pasture_data((uint8_t *)&pasture_temp, sizeof(pasture_temp));
		if (err) {
			return err;
		}
		return DOWNLOAD_COMPLETE;
	}

	return frame;
}

/**
 * @brief Process an incomming ANO message, mainly stores it to external flash.
 * @param anoResp The ANO message.
 * @return Returns 0 if successfull, otherwise negative error code.
 */
uint8_t process_ano_msg(UbxAnoReply *anoResp)
{
	uint8_t rec_ano_frames = anoResp->rgucBuf.size / sizeof(UBX_MGA_ANO_RAW_t);

	UBX_MGA_ANO_RAW_t *temp = NULL;
	temp = (UBX_MGA_ANO_RAW_t *)(anoResp->rgucBuf.bytes + sizeof(UBX_MGA_ANO_RAW_t));

	uint32_t age = ano_date_to_unixtime_midday(temp->mga_ano.year, temp->mga_ano.month,
						   temp->mga_ano.day);

	/* Write to storage controller's ANO WRITE partition. */
	int err = stg_write_ano_data((uint8_t *)&anoResp->rgucBuf,
				     anoResp->rgucBuf.size);

	if (err) {
		LOG_ERR("Error writing ano frame to storage controller (%d)", err);
		nf_app_error(ERR_MESSAGING, err, NULL, 0);
	}
	int64_t current_time_ms = 0;

	err = date_time_now(&current_time_ms);
	if (err) {
		LOG_ERR("Error fetching date time (%d)", err);
		nf_app_error(ERR_MESSAGING, err, NULL, 0);
	}
	if (age > (current_time_ms / 1000) + SECONDS_IN_THREE_DAYS) {
		return DOWNLOAD_COMPLETE;
	}
	return rec_ano_frames;
}

/**
 * @brief Obtaines the last know date pos.
 * @param gpsLastFix The latest GNSS fix.
 * @param pos The resulting date pos.
 */
static void proto_get_last_known_date_pos(gnss_last_fix_struct_t *gpsLastFix, _DatePos *pos)
{
	bool valid_headVeh = (bool)(gpsLastFix->pvt_flags & GPS_UBX_NAV_PVT_VALID_HEADVEH_MASK);
	_DatePos a = {
		.lLat = gpsLastFix->lat,
		.lLon = gpsLastFix->lon,
		.usHorizontalAccDm = gpsLastFix->h_acc_dm,
		.ulUnixTimestamp = gpsLastFix->unix_timestamp,
		.has_sHeadVeh = valid_headVeh,
		.sHeadVeh = gpsLastFix->head_veh,
		.has_usHeadAcc = valid_headVeh,
		.usHeadAcc = gpsLastFix->head_acc,
		.has_ucNumSV = true,
		.ucNumSV = gpsLastFix->num_sv,
		.has_usHDOP = true,
		.usHDOP = gpsLastFix->h_dop,
		.has_lHeight = true,
		.lHeight = gpsLastFix->height,
#if defined(HW_BAROMETER)
		.has_usHeight = true,
		.usHeight = gpsLastFix->baro_Height,
#endif
		.has_ucGpsMode = true,
		.ucGpsMode = gpsLastFix->mode,
		.has_xNavPlExtra = true,
		.xNavPlExtra = {
			.ucTmirCoeff = gpsLastFix->pl.tmirCoeff,
			.ucTmirExp = gpsLastFix->pl.tmirExp,
			.ucPlPosValid = gpsLastFix->pl.plPosValid,
			.ucPlPosFrame = gpsLastFix->pl.plPosFrame,
			.ulPlPos1 = gpsLastFix->pl.plPos1,
			.ulPlPos2 = gpsLastFix->pl.plPos2,
			.ulPlPos3 = gpsLastFix->pl.plPos3
		}

	};
	memcpy(pos, &a, sizeof(_DatePos));
}

/**
 * @brief Obtains whether or not there is a valid date pos.
 * @param gps The latest GNSS fix.
 * @return Returns true if there is a valid date pos, otherwise false.
 */
bool proto_has_last_known_date_pos(const gnss_last_fix_struct_t *gps)
{
	return gps->unix_timestamp != 0;
}

/**
 * @brief Converts ANO datetime to unix time.
 * @param year ANO data year.
 * @param month ANO data month.
 * @param day ANO data day.
 * @return Returns the unit time.
 */
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

/**
 * @brief Sets the initial collar state flag indicating that the module has recieved a collar state.
 * 		  The initial collar state flags are only set once for system startup sequence.
 * @param[in] uint8_t A collar state flag indicator (see collar_state_flags).
 */
static inline void set_initial_collar_state_flag(uint8_t aFlag) {
    if (aFlag >= COLLAR_STATE_FLAG_CNT) {
        return;
    }
    m_initial_collar_state_flags |= (1 << aFlag);
}

/**
 * @brief Checks whether the module has recieved all initial collar states (see collar_state_flags).
 * @return Return true if all initial collar states has been received, otherwise false.
 */
static inline bool has_initial_collar_states() {
    return (m_initial_collar_state_flags == ((1 << COLLAR_MODE_FLAG) | (1 << COLLAR_STATUS_FLAG) |
    		(1 << FENCE_STATUS_FLAG) | (1 << BATTERY_LVL_FLAG)) ? true : false);
}
