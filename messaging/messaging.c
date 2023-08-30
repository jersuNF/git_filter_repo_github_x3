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
#include <sys/timeutil.h>
#include "amc_const.h"
#include "pwr_event.h"
#include "nofence_watchdog.h"
#include "timeutil.h"
#include "nclogs.h"

#ifdef NRF52840_XXAA
#include <nrfx_nvmc.h>
#endif

#include "coredump_header.h"
#include <sys/util.h>

#define MODULE messaging
LOG_MODULE_REGISTER(MODULE, CONFIG_MESSAGING_LOG_LEVEL);

#define DOWNLOAD_COMPLETE 255
#define GPS_UBX_NAV_PVT_VALID_HEADVEH_MASK 0x20
#define TWO_AND_A_HALF_DAYS_SEC (3600 * (24 + 24 + 12))
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
int32_t cached_temp = 0;
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

typedef struct {
	Mode collar_mode;
	CollarStatus collar_status;
	FenceStatus fence_status;
	enum pwr_state_flag pwr_state;
	uint32_t fence_version;
	uint16_t flash_erase_count;
	uint16_t zap_count;
} collar_state_struct_t;

static collar_state_struct_t current_state;
static gnss_last_fix_struct_t cached_fix;

static bool block_fota_request = false;
static uint8_t m_upload_periodic_logs = 1;
static uint8_t m_upload_core_dumps = 1;

/** @brief counts the number of FOTA requests, only reset on success */
static int m_fota_attempts = 0;

typedef enum {
	COLLAR_MODE,
	COLLAR_STATUS,
	FENCE_STATUS,
	FENCE_VERSION,
	/** @todo Not written to storage, should it? -> FLASH_ERASE_COUNT,*/
	ZAP_COUNT,
	GNSS_STRUCT,
	GSM_INFO,
	MODEM_READY,
	CACHED_READY_END_OF_LIST
} cached_and_ready_enum;

/* Used to check if we've cached what is requried before we perform the
 * INITIAL, FIRST poll request to server */
static int cached_and_ready_reg[CACHED_READY_END_OF_LIST];

static int rat, mnc, rssi, min_rssi, max_rssi;
static uint8_t ccid[20] = "\0";
static char mdm_fw_file_name[sizeof(((PollMessageResponse *)NULL)->xModemFwFileName)] = "\0";
static char urat_in_use_buf[STG_CONFIG_URAT_ARG_BUF_SIZE] = "\0";

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

int request_ano_frame(uint16_t, uint16_t);

static void ano_download_work_fn(struct k_work *item);

void proto_InitHeader(NofenceMessage *);

void process_poll_response(NofenceMessage *);

static int process_upgrade_request(VersionInfoFW *);

uint8_t process_fence_msg(FenceDefinitionResponse *);

static int process_ano_msg(UbxAnoReply *anoResp);

int encode_and_send_message(NofenceMessage *);

int encode_and_store_message(NofenceMessage *);

int send_binary_message(uint8_t *, size_t);

static int send_all_stored_messages(void);

static void proto_get_last_known_date_pos(gnss_last_fix_struct_t *gpsLastFix, _DatePos *pos);

bool proto_has_last_known_date_pos(const gnss_last_fix_struct_t *);

void messaging_rx_thread_fn(void);

void messaging_tx_thread_fn(void);

// Transfer boot parameters in the first poll request after start up.
static bool m_transfer_boot_params = true;

static bool m_confirm_acc_limits, m_confirm_ble_key;
static bool m_confirm_urat_arg = false;
static bool m_ACK_bSendPeriodicLogs = false;
static bool m_ACK_bSendCoreDumps = false;
static bool m_ACK_xLogConfig = false;

#ifdef CONFIG_STG_CONFIG_DEBUG_SEND_WRITE_ERRORS
extern uint16_t g_nvs_write_errors;
static uint16_t m_sent_nvs_errors;
#endif

#ifdef CONFIG_STG_CONFIG_DEBUG_SEND_WRITE_ERRORS
extern uint16_t g_nvs_write_errors;
static uint16_t m_sent_nvs_errors;
#endif

K_MUTEX_DEFINE(send_binary_mutex);
K_MUTEX_DEFINE(read_flash_mutex);
static bool reboot_scheduled = false;

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
struct k_work_delayable seq_periodic_work;
struct k_work_delayable data_request_work;
struct k_work_delayable seq_message_work;
struct k_work_delayable process_escape_work;
struct k_work_delayable process_zap_work;
struct k_work_delayable process_warning_work;
struct k_work_delayable process_warning_correction_start_work;
struct k_work_delayable process_warning_correction_end_work;
struct k_work_delayable seq_message_send_work;
struct k_work_delayable fota_wdt_work;
struct k_work_delayable ano_download_work;

struct fence_def_update {
	struct k_work_delayable work;
	int version;
	int request_frame;
} m_fence_update_req;

static struct store_pos_update {
	PositionType pos_type;
	struct k_work_delayable work;
} store_pos_work_container;

static atomic_t poll_period_seconds = ATOMIC_INIT(CONFIG_DEFAULT_POLL_INTERVAL_MINUTES * 60);
static atomic_t seq_period_min = ATOMIC_INIT(CONFIG_DEFAULT_SEQ_INTERVAL_MINUTES);
static atomic_t m_new_mdm_fw_update_state = ATOMIC_INIT(0);

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
	SEQ_MSG /* Send stored seq messages */,
	FENCE_REQ /* Send a fence update request */
	/* Add additional states here (ANO, diagnostic etc)... */
} messaging_tx_type_t;

atomic_t m_fota_in_progress = ATOMIC_INIT(0);
atomic_t m_break_seq_stream_token = ATOMIC_INIT(0);

atomic_t m_message_tx_type = ATOMIC_INIT(0);

#define WDT_MODULE_MESSAGING ("messaging")
#define WDT_MODULE_KEEP_ALIVE ("keep_alive")
#define WDT_MODULE_RECV_TCP ("receive_tcp")

static int set_tx_state_ready(messaging_tx_type_t tx_type);

/**
 * @brief, called when the external flash has been erased, in case we
 * must reset the ANO variables in NVS
 */
static void process_flash_erased(void)
{
	int err;
	NCLOG_DBG(MESSAGING_MODULE, TRice0( iD( 5519),"dbg: Clearing ANO config values\n"));
	err = stg_config_u16_write(STG_U16_ANO_ID, UINT16_MAX);
	err |= stg_config_u16_write(STG_U16_ANO_START_ID, UINT16_MAX);
	err |= stg_config_u16_write(STG_U16_LAST_GOOD_ANO_ID, 0);
	err |= stg_config_u32_write(STG_U32_ANO_TIMESTAMP, 0);

	if (err) {
		NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 5572),"err: Could not clear stg %d\n", err));
	}
}

/**
 * @brief: Checks if we should kick-off a new ano download sequence.
 * Uses our serial number to smear out requests so that collars do not
 * bomb the server at the same time
 */
static void check_kickoff_ano_download_start()
{
	uint32_t ano_timestamp;
	uint16_t offset = (serial_id & 0x0FF) * 56;
	int ret = stg_config_u32_read(STG_U32_ANO_TIMESTAMP, &ano_timestamp);
	if (ret != 0 || ano_timestamp == UINT32_MAX) {
		NCLOG_WRN(MESSAGING_MODULE, TRice( iD( 2897),"wrn: Cannot read last ano timestamp %d\n", ret));
		ano_timestamp = 0;
	}
	int64_t curr_time;
	if (date_time_now(&curr_time) != 0) {
		NCLOG_DBG(MESSAGING_MODULE, TRice0( iD( 6478),"dbg: No current time, cannot download ANO now\n"));
		return;
	}
	uint32_t unix_time = (uint32_t)(curr_time / 1000);
	if (ano_timestamp + offset <= unix_time + (3600 * 24 * 3)) {
		ret = k_work_reschedule_for_queue(&message_q, &ano_download_work, K_NO_WAIT);
		if (ret < 0) {
			NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 1145),"err: Failed to schedule ANO\n"));
		}
	} else {
		NCLOG_DBG(MESSAGING_MODULE, TRice( iD( 3116),"dbg: No download needed: ano_t: %lu, offset: %lu, unix_time %lu\n", (unsigned long)ano_timestamp, (unsigned long)offset, (unsigned long)unix_time));
	}
}

/**
 * @brief Builds SEQ messages (1 and 2) with the latest data and store them to external storage.
 */
static int build_seq_message()
{
	int err;

	/* Fetch histogram data */
	struct save_histogram *histogram_snapshot = new_save_histogram();
	EVENT_SUBMIT(histogram_snapshot);

	collar_histogram histogram;
	err = k_msgq_get(&histogram_msgq, &histogram, K_SECONDS(10));
	if (err) {
		NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 5054),"err: Timeout on waiting for histogram %d\n", err));
		return err;
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
	seq_msg.m.seq_msg.usChargeMah =
		(uint16_t)(cached_chrg * CONFIG_CHARGING_POLLER_WORK_MSEC / MSECCONDS_PER_HOUR);
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
		NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 3515),"err: Failed to encode and save sequence message 1\n"));
		return err;
	}

	/* Build seq 2 message */
	seq_msg.which_m = NofenceMessage_seq_msg_2_tag;
	seq_msg.m.seq_msg_2.has_bme280 = true;
	seq_msg.m.seq_msg_2.bme280.ulPressure = (uint32_t)atomic_get(&cached_press);
	seq_msg.m.seq_msg_2.bme280.ulTemperature = cached_temp;
	seq_msg.m.seq_msg_2.bme280.ulHumidity = (uint32_t)atomic_get(&cached_hum);
	seq_msg.m.seq_msg_2.has_xBatteryQc = true;
	seq_msg.m.seq_msg_2.xBatteryQc.usVbattMax = histogram.qc_battery.usVbattMax;
	seq_msg.m.seq_msg_2.xBatteryQc.usVbattMin = histogram.qc_battery.usVbattMin;
	seq_msg.m.seq_msg_2.xBatteryQc.usTemperature = 0;
	seq_msg.m.seq_msg_2.has_xGnssModeCounts = true;
	memcpy(&seq_msg.m.seq_msg_2.xGnssModeCounts, &histogram.gnss_modes,
	       sizeof(seq_msg.m.seq_msg_2.xGnssModeCounts));

	/* Store Seq 2 message to non-volatile storage */
	err = encode_and_store_message(&seq_msg);
	if (err != 0) {
		NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 1774),"err: Failed to encode and save sequence message 2\n"));
	}
	return err;
}

/**
 * @brief Callback passed to the storage controller to send seq messages stored to external flash.
 * @param data Encoded seq messages read from storage.
 * @param len Length of the encoded seq message read from storage.
 * @return Returns 0 if successfull, otherwise negative error code.
 */
static int read_and_send_seq_data_cb(uint8_t *data, size_t len)
{
	/* Only send seq data stored to flash if not halted by some other process, e.g. a pending
     * FOTA. Retuning an error from this callback will abort the FCB walk in the storage
     * controller untill seq data trafic is reinstated. */
	if (atomic_get(&m_fota_in_progress) == true) {
		NCLOG_DBG(MESSAGING_MODULE, TRice0( iD( 7836),"dbg: FOTA download in progress, will not send log data now!\n"));
		return -EBUSY;
	}

	if (atomic_get(&m_break_seq_stream_token) == true) {
		NCLOG_DBG(MESSAGING_MODULE, TRice0( iD( 6058),"dbg: Breaking the seq stream!\n"));
		return -EBUSY;
	}

	NCLOG_DBG(MESSAGING_MODULE, TRice0( iD( 3413),"dbg: Send seq message fetched from flash\n"));

	/* Fetch the length from the two first bytes */
	uint16_t new_len = *(uint16_t *)&data[0];

	int err = send_binary_message(data, new_len);
	if (err) {
		NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 6448),"err: Error sending binary message for seq data %d\n", err));
	}
	return err;
}

/**
 * @brief Sends all seq messages stored to external flash, see read_and_send_seq_data_cb.
 * @return Returns 0 if successfull, otherwise negative error code.
 */
static int send_all_stored_messages(void)
{
	k_mutex_lock(&read_flash_mutex, K_NO_WAIT);
	if (read_flash_mutex.lock_count == 1) {
		/*Read and send out all the seq data if any.*/
		int err = stg_read_seq_data(read_and_send_seq_data_cb, 0);
		if (err && err != -ENODATA) {
			k_mutex_unlock(&read_flash_mutex);
			NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 2660),"err: stg_read_seq_data error: %i\n", err));
			return err;
		} else if (err == -ENODATA) {
			NCLOG_INF(MESSAGING_MODULE, TRice0( iD( 1247),"inf: No seq data available on flash for sending.\n"));
		}

		/* If all entries has been consumed, empty storage and we HAVE data on the
         * partition.*/
		if (stg_seq_pointing_to_last()) {
			err = stg_clear_partition(STG_PARTITION_SEQ);
			if (err) {
				NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 6099),"err: Error clearing FCB storage for SEQ %i\n", err));
				k_mutex_unlock(&read_flash_mutex);
				return err;
			} else {
				NCLOG_INF(MESSAGING_MODULE, TRice0( iD( 7699),"inf: Emptied SEQ partition data as we have read everything.\n"));
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
 * @brief Work item handler for "seq_periodic_work". Builds seq messages and store them to
 * external flash, and schedules an immediate send.
 * Rescheduled at regular interval as set by "seq_period_min".
 */
static void seq_periodic_fn()
{
	int ret;
	ret = k_work_reschedule_for_queue(&message_q, &seq_periodic_work,
					  K_MINUTES(atomic_get(&seq_period_min)));
	if (ret < 0) {
		NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 7508),"err: Failed to reschedule periodic seq messages!\n"));
	}

	// Only build and send SEQ messages if the collar is in normal power state
	if (current_state.pwr_state < PWR_NORMAL) {
		return;
	}

	ret = build_seq_message();
	NCLOG_DBG(MESSAGING_MODULE, TRice0( iD( 1326),"dbg: SEQ messages stored to flash\n"));

	if (m_transfer_boot_params) {
		/* Do not send SEQ message before startup poll request is sent to server */
		return;
	}

	/* Schedule work to send seq messages immediately */
	ret = k_work_reschedule_for_queue(&message_q, &seq_message_send_work, K_NO_WAIT);
	if (ret < 0) {
		NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 1905),"err: Failed to schedule a send of periodic seq messages!\n"));
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
		NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 6485),"err: Failed to reschedule periodic poll request!\n"));
	}

	/* Attempt to send poll request immediately */
	ret = set_tx_state_ready(POLL_REQ);
	if (ret != 0) {
		NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 4326),"err: Periodic poll failed, error %d\n", ret));
	}

	static bool initialized = false;
	if (!initialized) {
		initialized = true;
		ret = k_work_schedule_for_queue(&message_q, &seq_periodic_work, K_MSEC(250));
		if (ret != 0) {
			NCLOG_DBG(MESSAGING_MODULE, TRice( iD( 6512),"dbg: Periodic seq failed, reschedule, error %d\n", ret));
		}
	}
}

/**
 * @brief Build and store a position message with the latest data.
 */
void store_position_message_fn(struct k_work *item)
{
	struct store_pos_update *container = CONTAINER_OF(item, struct store_pos_update, work);
	NofenceMessage msg;
	proto_InitHeader(&msg);
	msg.which_m = NofenceMessage_position_msg_tag;
	// pos msg specifics
	msg.m.position_msg.eType = container->pos_type;
	proto_get_last_known_date_pos(&cached_fix, &msg.m.position_msg.xDatePos);

	int ret = encode_and_store_message(&msg);
	if (ret != 0) {
		NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 2324),"err: Failed to encode and store position message\n"));
		return;
	}
}

/**
 * @brief Work item handler for "seq_message_send_work". Initiate and sets Tx ready. Reschedules the
 * sending of seq data if Tx thread is busy.
 */
void seq_message_send_work_fn()
{
	static uint8_t retry_cnt = 0;

	int err = set_tx_state_ready(SEQ_MSG);
	if (err != 0 && retry_cnt++ <= 2) {
		NCLOG_DBG(MESSAGING_MODULE, TRice0( iD( 4339),"dbg: Unable to schedule seq messages, Tx thread not ready- rescheduling\n"));
		err = k_work_reschedule_for_queue(&message_q, &seq_message_send_work,
						  K_SECONDS(15));
		if (err < 0) {
			NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 3945),"err: Failed to reschedule work\n"));
		}
	} else {
		retry_cnt = 0;
	}
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
		NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 7179),"err: Failed to encode and store AMC correction ZAP message\n"));
		return;
	}
	NCLOG_DBG(MESSAGING_MODULE, TRice0( iD( 4035),"dbg: AMC Correction ZAP message stored to flash, scheduling immediate send\n"));

	/* Schedule work to send seq messages immediately */
	err = k_work_reschedule_for_queue(&message_q, &seq_message_send_work, K_NO_WAIT);
	if (err < 0) {
		NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 4659),"err: Failed to reschedule work\n"));
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
		NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 1249),"err: Failed to encode and store AMC escaped message\n"));
		return;
	}
	NCLOG_DBG(MESSAGING_MODULE, TRice0( iD( 5414),"dbg: AMC Escaped message stored to flash, scheduling immediate send\n"));

	/* Schedule work to send seq messages immediately */
	err = k_work_reschedule_for_queue(&message_q, &seq_message_send_work, K_NO_WAIT);
	if (err < 0) {
		NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 1545),"err: Failed to reschedule work\n"));
	}
}

/**
 * @brief Work item handler for "seq_message_work". Builds a status message and stores it to
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
		NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 5366),"err: Failed to encode and store AMC status message\n"));
		return;
	}
	NCLOG_DBG(MESSAGING_MODULE, TRice0( iD( 3723),"dbg: AMC Status message stored to flash, scheduling immediate send\n"));

	/* Schedule work to send seq messages immediately */
	err = k_work_reschedule_for_queue(&message_q, &seq_message_send_work, K_NO_WAIT);
	if (err < 0) {
		NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 2519),"err: Failed to reschedule work\n"));
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
		NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 7953),"err: Failed to encode and store AMC correction warning message\n"));
		return;
	}
	NCLOG_DBG(MESSAGING_MODULE, TRice0( iD( 5999),"dbg: AMC Correction warning message stored to flash\n"));
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
		NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 6157),"err: Failed to encode and store AMC correction start message\n"));
		return;
	}
	NCLOG_DBG(MESSAGING_MODULE, TRice0( iD( 6620),"dbg: AMC Correction start message stored to flash\n"));
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
		NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 3902),"err: Failed to encode and store AMC correction end message\n"));
		return;
	}
	NCLOG_DBG(MESSAGING_MODULE, TRice0( iD( 7037),"dbg: AMC Correction end message stored to flash\n"));
}

/**
 * @brief Work item handler for "m_fence_update_req.work". Schedules an immediate send of a fence
 * update request message.
 * @param item Pointer to work item (currently unused).
 */
void fence_update_req_fn(struct k_work *item)
{
	/* TODO, PSH, the retry should be removed once we
	 * have figured out why the schedule often fails on
	 * the first try */
	static int retry_cnt = 0;

	int ret = set_tx_state_ready(FENCE_REQ);
	if ((ret != 0) && (retry_cnt < 2)) {
		NCLOG_WRN(MESSAGING_MODULE, TRice0( iD( 4330),"wrn: Unable to schedule fence update req., Tx thread not ready, rescheduling\n"));
		ret = k_work_reschedule_for_queue(&message_q, &m_fence_update_req.work,
						  K_SECONDS(15));
		if (ret < 0) {
			NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 7290),"err: Failed to reschedule work\n"));
		}
		retry_cnt++;
		return;
	} else if ((ret != 0) && (retry_cnt >= 2)) {
		NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 7630),"err: Unable to schedule fence update req., exhausted retry attempts\n"));
		retry_cnt = 0;
	}
	return;
}

/**
 * @brief Send a log position message to the server before GNSS SLEEP and after GNSS wakeup,
 * to get some kind of qualitative measurements of “pulse in pasture”.
 */
static void log_position_state_machine()
{
	enum logpos_states { STARTUP = 0, WAIT_FOR_GNSS_INACTIVE, WAIT_FOR_FIX };
	static enum logpos_states active_state = STARTUP;
	static uint64_t last_pos_timestamp = 0;
	int ret;

	switch (active_state) {
	case STARTUP:
		if (cached_fix.unix_timestamp > 0) {
			active_state = WAIT_FOR_GNSS_INACTIVE;
		}
		break;
	case WAIT_FOR_GNSS_INACTIVE:
		if (cached_fix.mode == GNSSMODE_INACTIVE) {
			/* Do not send position log message inside beacon contact range. */
			if (current_state.fence_status != FenceStatus_BeaconContact &&
			    current_state.fence_status != FenceStatus_BeaconContactNormal) {
				store_pos_work_container.pos_type = PositionType_BEFORE_BBRAM;
				ret = k_work_schedule_for_queue(
					&message_q, &store_pos_work_container.work, K_NO_WAIT);
				if (ret < 0) {
					NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 1196),"err: Failed to send position log msg before GNSS SLEEP!\n"));
				}
				last_pos_timestamp = cached_fix.unix_timestamp;
			}
			active_state = WAIT_FOR_FIX;
		}
		break;
	case WAIT_FOR_FIX: {
		if (cached_fix.mode != GNSSMODE_INACTIVE) {
			if (cached_fix.unix_timestamp > last_pos_timestamp) {
				store_pos_work_container.pos_type = PositionType_AFTER_BBRAM;
				ret = k_work_reschedule_for_queue(
					&message_q, &store_pos_work_container.work, K_NO_WAIT);
				if (ret < 0) {
					NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 2028),"err: Failed to send position log msg after GNSS wake-up!\n"));
				}
				last_pos_timestamp = cached_fix.unix_timestamp;
				active_state = WAIT_FOR_GNSS_INACTIVE;
			}
		}
	} break;
	default:
		active_state = STARTUP;
		break;
	}
}
/**
 * @brief read out the core dumps and send them to the server in the form of protobuf GenericMessages.
 * @return Returns 0 if successfull, otherwise a negative error code.
 */
#ifdef CONFIG_DEBUG_COREDUMP_BACKEND_NOINIT
extern struct coredump_log __noinit dump;
static int coredump_storage_read_send(NofenceMessage *cd_msg)
{
	if (dump.magic_flag != CONFIG_CORE_DUMP_FLASH_MAGIC_FLAG) {
		return 0;
	}
	int failure = -EEXIST;
	size_t remaining_bytes = dump.ptr;
	size_t bytes_per_element = sizeof(cd_msg->m.generic_msg.usBuf[0]);
	size_t bytes_per_chunk = sizeof(cd_msg->m.generic_msg.usBuf);
	size_t total_chunks = ceiling_fraction(remaining_bytes, bytes_per_chunk);
	uint8_t chunk_id = 0;
	dump.ptr = 0; /* reset pointer to the beginning of the buffer we want to read from */

	while (remaining_bytes > 0) {
		size_t bytes2copy = MIN(remaining_bytes, bytes_per_chunk);
		if (bytes2copy < sizeof(uint32_t)) {
			NCLOG_WRN(MESSAGING_MODULE, TRice0( iD( 4538),"wrn: coredump is not word aligned.\n"));
		}

		proto_InitHeader(cd_msg);
		cd_msg->which_m = NofenceMessage_generic_msg_tag;
		cd_msg->m.generic_msg.has_usTotalChunks = true;
		cd_msg->m.generic_msg.usTotalChunks = total_chunks;
		cd_msg->m.generic_msg.msgType = GenericMessage_GenMessageType_CORE_DUMP;
		cd_msg->m.generic_msg.usChunkId = chunk_id;
		cd_msg->m.generic_msg.usBuf_count = ceiling_fraction(bytes2copy, bytes_per_element);
		memcpy(&cd_msg->m.generic_msg.usBuf[0], &dump.raw_data[dump.ptr], bytes2copy);

		failure = encode_and_send_message(cd_msg);
		if (failure) {
			NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 5368),"err: Failed to send core dump, err: %d!\n", failure));
			return failure;
		}

		remaining_bytes -= bytes2copy;
		dump.ptr += bytes2copy;
		chunk_id++;
	}
	dump.magic_flag = 0;
	return failure;
}
#endif

/**
 * @brief read out the trice encoded log messages from the dedicated buffer and
 * send them to the server in the form of protobuf GenericMessages.
 * @return Returns 0 if successfull, otherwise a negative error code.
 */
int send_trice_logs(NofenceMessage *trice_log_msg)
{
	uint8_t chunk_id = 0;
	int ret;
	int bytes_available = nclog_get_available_bytes();
	if (bytes_available < 0) {
		return -ENODATA;
	}
	size_t bytes_per_element = sizeof(trice_log_msg->m.generic_msg.usBuf[0]);
	size_t bytes_per_chunk = sizeof(trice_log_msg->m.generic_msg.usBuf);
	uint16_t total_chunks = ceiling_fraction(bytes_available, bytes_per_chunk);

	while (chunk_id * bytes_per_chunk <= CONFIG_NCLOG_BUFFER_SIZE) {
		proto_InitHeader(trice_log_msg);
		trice_log_msg->which_m = NofenceMessage_generic_msg_tag;
		trice_log_msg->m.generic_msg.has_usTotalChunks = true;
		trice_log_msg->m.generic_msg.usTotalChunks = total_chunks;
		trice_log_msg->m.generic_msg.usChunkId = chunk_id;

		trice_log_msg->m.generic_msg.msgType = GenericMessage_GenMessageType_TRICE_LOGS;

		ret = nclogs_read((uint8_t *)trice_log_msg->m.generic_msg.usBuf,
				  sizeof(trice_log_msg->m.generic_msg.usBuf));

		if (ret == 0) {
			NCLOG_INF(MESSAGING_MODULE, TRice0( iD( 5854),"inf: Uploaded all buffered logs!\n"));
			return 0;
		} else if (ret < 0) {
			NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 7113),"err: Error %d when reading buffered logs!\n", ret));
			return ret;
		} else {
			trice_log_msg->m.generic_msg.usBuf_count =
				ceiling_fraction(ret, bytes_per_element);
			ret = encode_and_send_message(trice_log_msg);
			if (ret != 0) {
				NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 7738),"err: Failed to send trice logs, err: %d!\n", ret));
				return ret;
			}
		}
		chunk_id++;
	}
	NCLOG_INF(MESSAGING_MODULE, TRice0( iD( 5589),"inf: Reached the limit for pushing logs!\n"));
	return 0;
}

/**
 * @brief Function that sets Tx type and starts the Tx sending sequence, if ready. Outgoing
 * messages from the messaging module are handled in a first come first serve manner- all though
 * all seq messages stored to flash are sent for each instance of SEQ_MSG.
 * @param tx_state Tx message type.
 * @param send_now Flag indicating whether to send message now or not.
 * @return Returns 0 if successfull, otherwise negative error code.
 */
static int set_tx_state_ready(messaging_tx_type_t tx_type)
{
	int state = atomic_get(&m_message_tx_type);
	if (state != IDLE) {
		/* Tx thread busy sending something else */
		if ((state == SEQ_MSG) && (tx_type == POLL_REQ)) {
			/* poll requests should always go through in the case of too many
             * seq messages stored on the flash. Tx thread will consume the
             * token when the fcb walk returns. */
			atomic_set(&m_break_seq_stream_token, true);
			return 0;
		}
		return -EBUSY;
	}
	if ((tx_type == SEQ_MSG)) {
		if (atomic_get(&m_fota_in_progress) == true) {
			/* Unable to send seq messages as seq data transfer is currently halted */
			return -EACCES;
		} else {
			atomic_set(&m_break_seq_stream_token, false);
		}
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
	while (true) {
		/* Block Tx thread untill semaphore is given */
		if (k_sem_take(&sem_release_tx_thread, K_FOREVER) == 0) {
			err = 0;
			messaging_tx_type_t tx_type = atomic_get(&m_message_tx_type);

			/* POLL REQUEST */
			if ((tx_type == POLL_REQ) || (m_last_poll_req_timestamp_ms == 0) ||
			    ((tx_type == SEQ_MSG) &&
			     ((k_uptime_get() - m_last_poll_req_timestamp_ms) >= 60000))) {
				/* For poll requests always check the connection before building the message.
				 * The message will be disregarded if the modem information is not available. */
				k_sem_reset(&cache_ready_sem);
				struct check_connection *ev = new_check_connection();
				EVENT_SUBMIT(ev);
				err = k_sem_take(
					&cache_ready_sem,
					K_MINUTES(CONFIG_COLLAR_STATES_CACHE_TIMEOUT_MINUTES));
				if (err == 0) {
					err = k_sem_take(&cache_lock_sem, K_SECONDS(1));
					if (err == 0) {
						NofenceMessage Nofence_msg_buffer;
						build_poll_request(&Nofence_msg_buffer);
						k_sem_give(&cache_lock_sem);

						err = encode_and_send_message(&Nofence_msg_buffer);
						if (err == 0) {
							/* Store poll req. timestamp to avoid sending an
                                                 * excessive amount of poll requests */
							m_last_poll_req_timestamp_ms =
								k_uptime_get();

#ifdef CONFIG_DEBUG_COREDUMP_BACKEND_NOINIT
							if (m_upload_core_dumps == 1) {
								err = coredump_storage_read_send(
									&Nofence_msg_buffer);

								if (err != 0 && err != -ENODATA) {
									NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 4613),"err: Failed to send stored core dump, %d!\n", err));
								}
							}
#endif
							// If there was no core dump (-ENODATA) still send trice logs.
							if ((err == 0 || err == -ENODATA) &&
							    (m_upload_periodic_logs == 1)) {
								err = send_trice_logs(
									&Nofence_msg_buffer);
								if (err != 0) {
									NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 2970),"err: Failed to send Trice logs, %d!\n", err));
								}
							}
						} else {
							LOG_WRN("Failed to send poll request!");
						}
					}
				} else {
					LOG_WRN("Cached semaphore not ready, dropping poll request!");
					NCLOG_WRN(MESSAGING_MODULE, TRice( iD( 2211),"wrn: Failed to send poll request err: %d\n", err));
					struct messaging_stop_connection_event *end_connection =
						new_messaging_stop_connection_event();
					EVENT_SUBMIT(end_connection);
				}
				/* Poll request error handler,
                                 * Note! Consider notifying sender, leaving error handling to src */
			}

			/* SEQ MESSAGES */
			if ((tx_type == SEQ_MSG) && (err == 0) &&
			    (atomic_get(&m_fota_in_progress) == false)) {
				/* Sending, all stored seq messages are already proto encoded */
				err = send_all_stored_messages();
				/* Seq message error handler,
                                 * Note! Consider notifying sender, leaving error handling to src */
				if (err != 0) {
					NCLOG_WRN(MESSAGING_MODULE, TRice0( iD( 4821),"wrn: Failed to send seq messages\n"));
				}
			}

			/* FENCE DEFINITION REQUEST */
			if ((tx_type == FENCE_REQ)) {
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
					NCLOG_WRN(MESSAGING_MODULE, TRice0( iD( 4161),"wrn: Failed to send fence update request\n"));
				}
			}

			/* Add additional message types here (system diagnostics, ANO data etc).. */

			/* Reset Tx thread */
			atomic_set(&m_message_tx_type, IDLE);

			if (atomic_get(&m_break_seq_stream_token) == true) {
				/* consume the token and enforce the poll request */
				atomic_set(&m_break_seq_stream_token, false);
				atomic_set(&m_message_tx_type, POLL_REQ);
				k_sem_give(&sem_release_tx_thread);
			}

		} else {
			NCLOG_WRN(MESSAGING_MODULE, TRice0( iD( 1455),"wrn: Tx thread semaphore returned unexpectedly\n"));
			k_sem_reset(&sem_release_tx_thread);
		}
	}
	NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 7405),"err: Messaging Tx Thread exited unexpectedly\n"));
}

/**
 * @brief Work item handler for "data_request_work". Sends a request for environment data, such
 * as temperatre, pressure and humidity. Rescheduled at a regular interval.
 */
void data_request_work_fn()
{
	NCLOG_INF(MESSAGING_MODULE, TRice0( iD( 2370),"inf: Periodic request of environment data\n"));
	int err = k_work_reschedule_for_queue(&message_q, &data_request_work, K_MINUTES(1));
	if (err < 0) {
		NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 4910),"err: Failed to reschedule work\n"));
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
	/* Always de-assert the modem ready bit */
	cached_and_ready_reg[MODEM_READY] = 0;
}

/**
 * @brief Main event handler function.
 * @param[in] eh Event_header for the if-chain to use to recognize which event triggered.
 * @return True if event is consumed, otherwise false.
 */
static bool event_handler(const struct event_header *eh)
{
	/* Kick watchdog here */
	nofence_wdt_kick(WDT_MODULE_MESSAGING);
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

		if (ev->gnss_data.fully_resolved_unix_timestamp != 0) {
			time_t gm_time = (time_t)ev->gnss_data.fully_resolved_unix_timestamp;
			struct tm *tm_time = gmtime(&gm_time);

			/* tm_year is relative to 1900 */
			if (tm_time->tm_year < 120) {
				NCLOG_WRN(MESSAGING_MODULE, TRice( iD( 3758),"wrn: Invalid gnss packet, unix time was %lu\n", (unsigned long)ev->gnss_data.lastfix.unix_timestamp));
				return false;
			}
			/* Update date_time library which storage uses for ANO data. */
			date_time_set(tm_time);
		}
		/* log_position state machine tick */
		log_position_state_machine();

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
		/* Kick receive_tcp() Watchdog */
		nofence_wdt_kick(WDT_MODULE_RECV_TCP);
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
				int err = k_work_reschedule_for_queue(&message_q, &seq_message_work,
								      K_NO_WAIT);
				if (err < 0) {
					NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 6825),"err: Failed to schedule seq status work %d\n", err));
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
				int err = k_work_reschedule_for_queue(&message_q, &seq_message_work,
								      K_NO_WAIT);
				if (err < 0) {
					NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 3833),"err: Failed to schedule seq message work %d\n", err));
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
				int err = k_work_reschedule_for_queue(&message_q, &seq_message_work,
								      K_NO_WAIT);
				if (err < 0) {
					NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 6964),"err: Failed to schedule seq message work %d\n", err));
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
			NCLOG_WRN(MESSAGING_MODULE, TRice0( iD( 2986),"wrn: Schedule poll request: fence_version!\n"));
			int err = k_work_reschedule_for_queue(&message_q, &modem_poll_work,
							      K_NO_WAIT);
			if (err < 0) {
				NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 5628),"err: Error schedule poll request work %d\n", err));
			}
		}
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
			NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 4146),"err: Error scheduling zap work %d\n", err));
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
			NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 2174),"err: Error reschedule escape work %d\n", err));
		}
		return false;
	}
	if (is_connection_state_event(eh)) {
		/* Kick watchdog here */
		nofence_wdt_kick(WDT_MODULE_KEEP_ALIVE);
		struct connection_state_event *ev = cast_connection_state_event(eh);
		if (ev->state) {
			k_sem_give(&connection_ready);
			update_cache_reg(MODEM_READY);
		} else {
			k_sem_reset(&connection_ready);
			k_sem_reset(&cache_ready_sem);
		}
		return false;
	}
	if (is_animal_warning_event(eh)) {
		struct animal_warning_event *ev = cast_animal_warning_event(eh);
		atomic_set(&cached_dist_warn, ev->fence_dist);

		int err = k_work_reschedule_for_queue(&message_q, &process_warning_work, K_NO_WAIT);
		if (err < 0) {
			NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 2953),"err: Error reschedule warning work %d\n", err));
		}
		return false;
	}
	if (is_pwr_status_event(eh)) {
		struct pwr_status_event *ev = cast_pwr_status_event(eh);
		// TODO: handle charging state properly
		if (ev->pwr_state != PWR_CHARGING) {
			/* We want battery voltage in deci volt */
			atomic_set(&cached_batt, (uint16_t)(ev->battery_mv / 10));
			set_initial_collar_state_flag(BATTERY_LVL_FLAG);
		} else {
			cached_chrg += ev->charging_ma;
		}

		current_state.pwr_state = ev->pwr_state;
		return false;
	}
	if (is_send_poll_request_now(eh)) {
		NCLOG_DBG(MESSAGING_MODULE, TRice0( iD( 6362),"dbg: Received a nudge on listening socket!\n"));
		int err;
		err = k_work_reschedule_for_queue(&message_q, &modem_poll_work, K_NO_WAIT);
		if (err < 0) {
			NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 7613),"err: Error starting modem poll worker in response to nudge on listening socket. %d\n", err));
		}
		return false;
	}
	if (is_env_sensor_event(eh)) {
		struct env_sensor_event *ev = cast_env_sensor_event(eh);

		NCLOG_DBG(MESSAGING_MODULE, TRice( iD( 7901),"dbg: Event Temp: %.2f, humid %.3f, press %.3f\n", ev->temp, ev->humidity, ev->press));

		/* Multiply sensor values with scaling factor and cache */
		atomic_set(&cached_press, (uint32_t)(ev->press * 1000));
		atomic_set(&cached_hum, (uint32_t)(ev->humidity * 1000));
		cached_temp = ev->temp * 100;
		return false;
	}
	if (is_warn_correction_start_event(eh)) {
		struct warn_correction_start_event *ev = cast_warn_correction_start_event(eh);
		atomic_set(&cached_dist_correction_start, ev->fence_dist);

		int err = k_work_reschedule_for_queue(
			&message_q, &process_warning_correction_start_work, K_NO_WAIT);
		if (err < 0) {
			NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 7971),"err: Error reschedule warning correction start work %d\n", err));
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
			NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 3032),"err: Error reschedule warning correction end work %d\n", err));
		}
		return false;
	}
	if (is_gsm_info_event(eh)) {
		struct gsm_info_event *ev = cast_gsm_info_event(eh);
		rat = ev->gsm_info.rat;
		mnc = ev->gsm_info.mnc;
		rssi = ev->gsm_info.rssi;
		min_rssi = ev->gsm_info.min_rssi;
		max_rssi = ev->gsm_info.max_rssi;
		memcpy(ccid, ev->gsm_info.ccid, sizeof(ccid));

		NCLOG_INF(MESSAGING_MODULE, TRice( iD( 1307),"inf: RSSI: %d, %d, %d\n", rssi, min_rssi, max_rssi));
		NCLOG_INF(MESSAGING_MODULE, TRice( iD( 4074),"inf: rat: %d, dynamic_string\n",rat));
		NCLOG_INF(MESSAGING_MODULE, TRice0( iD( 3796),"inf: ccid: dynamic_string\n"));

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

		if (fw_upgrade_event->dfu_status == DFU_STATUS_IDLE ||
		    fw_upgrade_event->dfu_status == DFU_STATUS_SUCCESS_REBOOT_SCHEDULED) {
			/* Error or cancelled, stop the watchdog */
			NCLOG_INF(MESSAGING_MODULE, TRice0( iD( 5998),"inf: Cancelling APP FOTA WDT\n"));
			k_work_cancel_delayable(&fota_wdt_work);
		} else {
			/* Start/Kick the FOTA application watchdog */
			NCLOG_INF(MESSAGING_MODULE, TRice0( iD( 2235),"inf: Kicking APP FOTA WDT\n"));
			k_work_reschedule_for_queue(&message_q, &fota_wdt_work,
						    K_MINUTES(CONFIG_APP_FOTA_WDT_MINUTES));
		}
		if (fw_upgrade_event->dfu_status == DFU_STATUS_IDLE &&
		    fw_upgrade_event->dfu_error != 0) {
			/* DFU/FOTA is canceled, release the halt on seq message traffic in
             * the messaging tx thread */
			NCLOG_WRN(MESSAGING_MODULE, TRice( iD( 4192),"wrn: DFU error %d\n", fw_upgrade_event->dfu_error));
			atomic_set(&m_fota_in_progress, false);
			if (m_fota_attempts > CONFIG_APP_FOTA_FAILURES_BEFORE_REBOOT) {
				int err = stg_config_u8_write(
					STG_U8_RESET_REASON,
					(uint8_t)REBOOT_FOTA_MAX_FAILURE_ATTEMPTS);
				if (err != 0) {
					NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 7203),"err: Error writing fota reset reason\n"));
				}
				NCLOG_WRN(MESSAGING_MODULE, TRice0( iD( 3845),"wrn: Rebooting due to too many failed FOTA tries\n"));
				sys_reboot(SYS_REBOOT_COLD);
			}
		} else if (fw_upgrade_event->dfu_status != DFU_STATUS_IDLE) {
			/* DFU/FOTA has started or is in progress, halt seq message traffic
             * in the messaging tx thread */
			atomic_set(&m_fota_in_progress, true);
		}
		return false;
	}
	if (is_mdm_fw_update_event(eh)) {
		struct mdm_fw_update_event *ev = cast_mdm_fw_update_event(eh);
		atomic_set(&m_new_mdm_fw_update_state, ev->status);
		if (ev->status == MDM_FW_DOWNLOAD_COMPLETE || ev->status == INSTALLATION_COMPLETE) {
			int err = k_work_reschedule_for_queue(&message_q, &modem_poll_work,
							      K_SECONDS(5));
			if (err < 0) {
				NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 2774),"err: Error starting modem poll worker on mdm fw update! %d\n", err));
			}
		}
		return false;
	}
	if (is_gnss_mode_changed_event(eh)) {
		const struct gnss_mode_changed_event *ev = cast_gnss_mode_changed_event(eh);
		cached_fix.mode = ev->mode;
		/* log_position state machine tick */
		log_position_state_machine();
		return false;
	}

	if (is_flash_erased_event(eh)) {
		current_state.flash_erase_count++;
		/** @todo Not written to storage. Should it? And also be added to
         * update cache reg??
         */
		process_flash_erased();
		return false;
	}
	if (is_urat_args_in_use_event(eh)) {
		struct urat_args_in_use_event *ev = cast_urat_args_in_use_event(eh);
		memset(urat_in_use_buf, 0, sizeof(urat_in_use_buf));
		strncpy(urat_in_use_buf, ev->urat_in_use, sizeof(urat_in_use_buf) - 1);
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
EVENT_SUBSCRIBE(MODULE, mdm_fw_update_event);
EVENT_SUBSCRIBE(MODULE, gnss_mode_changed_event);
EVENT_SUBSCRIBE(MODULE, start_ano_download);
EVENT_SUBSCRIBE(MODULE, flash_erased_event);
EVENT_SUBSCRIBE(MODULE, urat_args_in_use_event);

/**
 * @brief Process commands recieved on the bluetooth interface, and performs the appropriate
 * actions accordingly.
 */
static inline void process_ble_cmd_event(void)
{
	struct ble_cmd_event ev;

	int err = k_msgq_get(&ble_cmd_msgq, &ev, K_NO_WAIT);
	if (err) {
		NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 4582),"err: Error getting ble_cmd_event %d\n", err));
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
		NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 7247),"err: Error getting lte_proto_event %d\n", err));
		return;
	}

	NofenceMessage proto;
	err = collar_protocol_decode(ev.buf + 2, ev.len - 2, &proto);

	struct messaging_ack_event *ack = new_messaging_ack_event();
	EVENT_SUBMIT(ack);
	if (err) {
		NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 2373),"err: Error decoding protobuf message %d\n", err));
		return;
	}

	if (proto.which_m == NofenceMessage_poll_message_resp_tag) {
		NCLOG_INF(MESSAGING_MODULE, TRice0( iD( 7430),"inf: Process poll reponse\n"));
		process_poll_response(&proto);
	} else if (proto.which_m == NofenceMessage_fence_definition_resp_tag) {
		uint8_t received_frame = process_fence_msg(&proto.m.fence_definition_resp);
		fence_download(received_frame);
	} else if (proto.which_m == NofenceMessage_ubx_ano_reply_tag) {
		err = process_ano_msg(&proto.m.ubx_ano_reply);
		if (err == 0) {
			err = k_work_reschedule_for_queue(&message_q, &ano_download_work,
							  K_NO_WAIT);
			if (err < 0) {
				NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 3714),"err: Failed to schedule ANO\n"));
			}
		} else {
			NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 6066),"err: Failed to download ANO : %d\n", err));
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

/**
 * @brief Default App watchdog callback
 */
static void fota_app_wdt_cb()
{
	int err = stg_config_u8_write(STG_U8_RESET_REASON, (uint8_t)REBOOT_FOTA_HANG);
	if (err != 0) {
		NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 5933),"err: Error writing fota reset reason\n"));
	}
	sys_reboot(SYS_REBOOT_COLD);
}

static void nofence_wdt_cb_trigger(uint8_t reason)
{
	int err = stg_config_u8_write(STG_U8_RESET_REASON, reason);
	if (err != 0) {
		NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 5426),"err: Error writing fota reset reason\n"));
	}
	sys_reboot(SYS_REBOOT_COLD);
}

static fota_wdt_cb g_wdt_cb = fota_app_wdt_cb;

/**
 * @brief Trigger called by fota_wdt_work
 */
void fota_app_wdt_trigger()
{
	if (g_wdt_cb) {
		g_wdt_cb();
	}
}

/**
 * @brief Register/overwrite fota app watchdog callback
 * @param fota wdt callback
 */
void fota_wdt_cb_register(fota_wdt_cb wdt_cb)
{
	g_wdt_cb = wdt_cb;
}

/**
 * @brief Work item handler for "fota_wdt_work". Restarts the system
 * @param item Pointer to work item.
 */
static void fota_wdt_work_fn(struct k_work *item)
{
	fota_app_wdt_trigger();
}

int messaging_module_init(void)
{
	NCLOG_INF(MESSAGING_MODULE, TRice0( iD( 1820),"inf: Initializing messaging module.\n"));

	/* 	Read parameters from the Non Volatile Storage */
	int err = stg_config_u32_read(STG_U32_UID, &serial_id);
	if (err != 0) {
		NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 7349),"err: Failed to read serial number from storage! %d\n", err));
		/* Fallback if read from storage fails */
		serial_id = 1;
	}

	err = stg_config_u8_read(STG_U8_M_UPLOAD_PERIODIC_LOGS, &m_upload_periodic_logs);
	if (err != 0) {
		NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 3442), "err: Error reading m_upload_periodic_logs from ext flash %d\n", err));
		/* Fallback if read from storage fails */
		m_upload_periodic_logs = 1;
	}
	err = stg_config_u8_read(STG_U8_M_UPLOAD_CORE_DUMPS, &m_upload_core_dumps);
	if (err != 0) {
		NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 2285), "err: Error reading m_upload_core_dumps from ext flash %d\n", err));
		/* Fallback if read from storage fails */
		m_upload_core_dumps = 1;
	}

	k_work_queue_init(&message_q);
	struct k_work_queue_config cfg = {
		.name = "message_q",
	};
	k_work_queue_start(&message_q, messaging_send_thread,
			   K_THREAD_STACK_SIZEOF(messaging_send_thread),
			   K_PRIO_COOP(CONFIG_MESSAGING_SEND_THREAD_PRIORITY), &cfg);

	k_work_init_delayable(&modem_poll_work, modem_poll_work_fn);
	k_work_init_delayable(&seq_periodic_work, seq_periodic_fn);
	k_work_init_delayable(&seq_message_send_work, seq_message_send_work_fn);
	k_work_init_delayable(&store_pos_work_container.work, store_position_message_fn);
	k_work_init_delayable(&seq_message_work, log_status_message_fn);
	k_work_init_delayable(&process_escape_work, log_animal_escaped_work_fn);
	k_work_init_delayable(&process_zap_work, log_zap_message_work_fn);
	k_work_init_delayable(&process_warning_work, log_warning_work_fn);
	k_work_init_delayable(&process_warning_correction_start_work, log_correction_start_work_fn);
	k_work_init_delayable(&process_warning_correction_end_work, log_correction_end_work_fn);
	k_work_init_delayable(&data_request_work, data_request_work_fn);
	k_work_init_delayable(&m_fence_update_req.work, fence_update_req_fn);
	k_work_init_delayable(&fota_wdt_work, fota_wdt_work_fn);
	k_work_init_delayable(&ano_download_work, ano_download_work_fn);

	memset(&pasture_temp, 0, sizeof(pasture_t));
	cached_fences_counter = 0;
	pasture_temp.m.us_pasture_crc = EMPTY_FENCE_CRC;

	/* Initialize nofence watchdog */
	nofence_wdt_init();
	nofence_wdt_register_cb(nofence_wdt_cb_trigger);
	nofence_wdt_module_register(WDT_MODULE_MESSAGING, REBOOT_WDT_RESET_MESSAGING,
				    CONFIG_WDT_MODULE_MESSAGING_TIME_SECONDS);
	nofence_wdt_module_register(WDT_MODULE_KEEP_ALIVE, REBOOT_WDT_RESET_KEEP_ALIVE,
				    CONFIG_WDT_MODULE_KEEP_ALIVE_TIME_SECONDS);
	nofence_wdt_module_register(WDT_MODULE_RECV_TCP, REBOOT_WDT_RESET_RECV_TCP,
				    CONFIG_WDT_MODULE_RECV_TCP_TIME_SECONDS);

	/** @todo Should add semaphore and only start these queues when
     *  we get connection to network with modem.
     */

	err = k_work_schedule_for_queue(&message_q, &modem_poll_work, K_NO_WAIT);
	if (err < 0) {
		return err;
	}

	err = k_work_schedule_for_queue(&message_q, &data_request_work, K_NO_WAIT);
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

	/* ACK the bSendPeriodicLogs once after it is updated in process_poll_response.
	   The server may update bSendPeriodicLogs to true or false,
	   an ACK to either update is signalled by replying with the same value as in the poll response. */
	if (m_ACK_bSendPeriodicLogs) {
		poll_req->m.poll_message_req.has_bSendPeriodicLogs = true;
		poll_req->m.poll_message_req.bSendPeriodicLogs = (bool)m_upload_periodic_logs;
	}

	/* ACK the bSendCoreDumps once after it is updated in process_poll_response.
	   The server may update bSendCoreDumps to true or false,
	   an ACK to either update is signalled by replying with the same value as in the poll response. */
	if (m_ACK_bSendCoreDumps) {
		poll_req->m.poll_message_req.has_bSendCoreDumps = true;
		poll_req->m.poll_message_req.bSendCoreDumps = (bool)m_upload_core_dumps;
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
			NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 4276),"err: Failed to read STG_U16_ACC_SIGMA_SLEEP_LIMIT %d\n", err));
		}

		err = stg_config_u16_read(STG_U16_ACC_SIGMA_NOACTIVITY_LIMIT,
					  &poll_req->m.poll_message_req.usAccSigmaNoActivityLimit);
		if (err != 0) {
			NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 5685),"err: Failed to read STG_U16_ACC_SIGMA_NOACTIVITY_LIMIT %d\n", err));
		}

		err = stg_config_u16_read(STG_U16_OFF_ANIMAL_TIME_LIMIT_SEC,
					  &poll_req->m.poll_message_req.usOffAnimalTimeLimitSec);
		if (err != 0) {
			NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 5190),"err: Failed to read STG_U16_OFF_ANIMAL_TIME_LIMIT_SEC %d\n", err));
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
			NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 5573),"err: Failed to read ble_sec_key %d\n", err));
		}
	}

#ifdef CONFIG_STG_CONFIG_DEBUG_SEND_WRITE_ERRORS
	/* If there are any NVS write errors, send them until reply */
	if (g_nvs_write_errors > 0) {
		poll_req->m.poll_message_req.has_usEepromErrors = true;
		poll_req->m.poll_message_req.usEepromErrors = g_nvs_write_errors;
		m_sent_nvs_errors = poll_req->m.poll_message_req.usEepromErrors;
	}
#endif

	poll_req->m.poll_message_req.has_xModemUratArg = true;
	strncpy(poll_req->m.poll_message_req.xModemUratArg, urat_in_use_buf,
		sizeof(poll_req->m.poll_message_req.xModemUratArg) - 1);

	/* TODO pshustad, fill GNSSS parameters for MIA M10 */
	poll_req->m.poll_message_req.has_usGnssOnFixAgeSec =
		(cached_gnss_mode == GNSSMODE_NOMODE) ? false : true;
	uint32_t timeSinceFixSec = (cached_msss - cached_fix.msss) / 1000;
	if (timeSinceFixSec > UINT16_MAX) {
		timeSinceFixSec = UINT16_MAX;
	}
	poll_req->m.poll_message_req.usGnssOnFixAgeSec = timeSinceFixSec;

	poll_req->m.poll_message_req.has_usGnssTTFFSec =
		(cached_gnss_mode == GNSSMODE_NOMODE) ? false : true;
	uint32_t timeSinceFirstFixSec = cached_ttff / 1000;
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
	}

	if (m_transfer_boot_params ||
	    atomic_get(&m_new_mdm_fw_update_state) >= MDM_FW_DOWNLOAD_COMPLETE) {
		/* Add modem model and FW version */
		const char *modem_model = NULL;
		const char *modem_version = NULL;
		int ret = modem_nf_get_model_and_fw_version(&modem_model, &modem_version);
		if (ret == 0) {
			poll_req->m.poll_message_req.has_xVersionInfoModem = true;
			strncpy(poll_req->m.poll_message_req.xVersionInfoModem.xModel, modem_model,
				sizeof(poll_req->m.poll_message_req.xVersionInfoModem.xModel) - 1);
			strncpy(poll_req->m.poll_message_req.xVersionInfoModem.xVersion,
				modem_version,
				sizeof(poll_req->m.poll_message_req.xVersionInfoModem.xVersion) -
					1);
			if (atomic_get(&m_new_mdm_fw_update_state) == MDM_FW_DOWNLOAD_COMPLETE) {
				poll_req->m.poll_message_req.xVersionInfoModem
					.has_xModemFwFileNameDownloaded = true;
				strncpy(poll_req->m.poll_message_req.xVersionInfoModem
						.xModemFwFileNameDownloaded,
					&mdm_fw_file_name[0],
					sizeof(poll_req->m.poll_message_req.xVersionInfoModem
						       .xModemFwFileNameDownloaded) -
						1);
				NCLOG_INF(MESSAGING_MODULE, TRice0( iD( 6121),"inf: dynamic_string\n"));
			}
		} else {
			NCLOG_WRN(MESSAGING_MODULE, TRice( iD( 4011),"wrn: Could not get modem version info: %d\n", ret));
		}
	}
	if (m_ACK_xLogConfig || m_transfer_boot_params) {
		poll_req->m.poll_message_req.has_xLogConfig = true;
		/* loop over each module and return the log level */
		for (size_t i = 0; i < _eNCLOG_MODULE_MAX; i++) {
			poll_req->m.poll_message_req.xLogConfig.xModule[i].xLevel =
				nclog_get_level(i);
			poll_req->m.poll_message_req.xLogConfig.xModule[i].xName = (eNCLOG_MODULE)i;
			poll_req->m.poll_message_req.xLogConfig.xModule_count++;
		}
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

		NCLOG_INF(MESSAGING_MODULE, TRice( iD( 1439),"inf: Fence ver %d download complete and notified AMC.\n", m_fence_update_req.version));
	} else if (received_frame == m_fence_update_req.request_frame) {
		m_fence_update_req.request_frame++;

		NCLOG_INF(MESSAGING_MODULE, TRice( iD( 6976),"inf: Requesting frame %d of new fence: %d\n", m_fence_update_req.request_frame, m_fence_update_req.version));

		/* Submit a fence frame message request */
		int err = k_work_reschedule_for_queue(&message_q, &m_fence_update_req.work,
						      K_NO_WAIT);
		if (err < 0) {
			NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 7718),"err: Failed to reschedule work\n"));
		}
	} else {
		/* Received incorrect fence frame number, cancel download */
		m_fence_update_req.version = current_state.fence_version;
		m_fence_update_req.request_frame = 0;
	}
}

/**
 * @brief Builds and sends a ANO data request to the server.
 * @return Returns 0 if successfull, otherwise a negative error code.
 */
int request_ano_frame(uint16_t ano_id, uint16_t ano_start)
{
	NCLOG_DBG(MESSAGING_MODULE, TRice0( iD( 1100),"dbg: request_ano_frame()\n"));
	NofenceMessage ano_req;
	proto_InitHeader(&ano_req); /* fill up message header. */
	ano_req.which_m = NofenceMessage_ubx_ano_req_tag;
	ano_req.m.ubx_ano_req.usAnoId = ano_id;
	ano_req.m.ubx_ano_req.usStartAno = ano_start;
	int ret = encode_and_send_message(&ano_req);
	if (ret) {
		NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 3690),"err: Failed to send request for ano %d %d\n", ano_start, ret));
		return -1;
	}
	return 0;
}

/**
 * @brief Handler for ANO data download.
 */
static void ano_download_work_fn(struct k_work *item)
{
	int64_t curr_time;
	uint32_t ano_timestamp;
	uint16_t ano_id;
	uint16_t ano_frame;
	if (date_time_now(&curr_time) != 0) {
		NCLOG_DBG(MESSAGING_MODULE, TRice0( iD( 3813),"dbg: No current time, cannot download ANO now\n"));
		return;
	}
	uint32_t unix_time = (uint32_t)(curr_time / 1000);
	int ret = stg_config_u32_read(STG_U32_ANO_TIMESTAMP, &ano_timestamp);
	if (ret != 0 || ano_timestamp == UINT32_MAX) {
		NCLOG_WRN(MESSAGING_MODULE, TRice( iD( 1731),"wrn: Cannot read last ano timestamp %d\n", ret));
		ano_timestamp = 0;
	}
	/* If the latest ano frome is too old, we need to download */
	NCLOG_DBG(MESSAGING_MODULE, TRice( iD( 3003),"dbg: ano-time %lu, unix_time %lu\n", (unsigned long)ano_timestamp, (unsigned long)unix_time));
	if (ano_timestamp <= unix_time + (3600 * 24 * 3)) {
		ret = stg_config_u16_read(STG_U16_ANO_ID, &ano_id);
		if (ret != 0 || ano_id == UINT16_MAX) {
			ano_id = 0;
		}
		ret = stg_config_u16_read(STG_U16_ANO_START_ID, &ano_frame);
		if (ret != 0 || ano_frame == UINT16_MAX) {
			ano_frame = 0;
		}
		ret = request_ano_frame(ano_id, ano_frame);
		if (ret != 0) {
			NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 5836),"err: Cannot request ANO frame %d\n", ret));
		}
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
	int64_t curr_time = 0;
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
	if (k_mutex_lock(&send_binary_mutex, K_MSEC(CONFIG_SEND_BINARY_MUTEX_TIMEOUT_MSEC)) == 0) {
		k_sem_reset(&connection_ready);
		struct check_connection *ev = new_check_connection();
		EVENT_SUBMIT(ev);

		int ret = k_sem_take(&connection_ready, K_FOREVER);
		if (ret != 0) {
			NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 5058),"err: Connection not ready, can't send message now! %d\n", ret));
			k_mutex_unlock(&send_binary_mutex);
			return ret;
		}
		uint16_t byteswap_size = BYTESWAP16(len - 2);
		memcpy(&data[0], &byteswap_size, 2);

		k_sem_reset(&send_out_ack);
		struct messaging_proto_out_event *msg2send = new_messaging_proto_out_event();
		msg2send->buf = data;
		msg2send->len = len;
		EVENT_SUBMIT(msg2send);

		ret = k_sem_take(&send_out_ack, K_FOREVER);
		if (ret != 0) {
			NCLOG_WRN(MESSAGING_MODULE, TRice0( iD( 1406),"wrn: Message not sent!\n"));
			/* todo: remove later, it is needed to pass the twister tests*/
			nf_app_error(ERR_MESSAGING, ret, NULL, 0);
			k_mutex_unlock(&send_binary_mutex);
			return ret;
		}
		k_mutex_unlock(&send_binary_mutex);
		return 0;
	} else {
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

	NCLOG_INF(MESSAGING_MODULE, TRice( iD( 6823),"inf: Start message encoding, tag: %u, version: %u\n", msg_proto->which_m, msg_proto->header.ulVersion));
	int ret = collar_protocol_encode(msg_proto, &encoded_msg[2], NofenceMessage_size,
					 &encoded_size);
	if (ret) {
		NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 3135),"err: Error encoding nofence message %d\n", ret));
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

	NCLOG_INF(MESSAGING_MODULE, TRice( iD( 7951),"inf: Start message encoding, tag: %u, version: %u\n", msg_proto->which_m, msg_proto->header.ulVersion));
	ret = collar_protocol_encode(msg_proto, &encoded_msg[2], NofenceMessage_size,
				     &encoded_size);
	if (ret) {
		NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 6253),"err: Error encoding nofence message %d\n", ret));
		return ret;
	}
	uint16_t total_size = encoded_size + header_size;

	/* Store the length of the message in the two first bytes */
	memcpy(&encoded_msg[0], &total_size, 2);

	ret = stg_write_seq_data(encoded_msg, (size_t)total_size);
	if (ret != 0) {
		NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 5871),"err: Failed to store message to flash!\n"));
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

	/* Transfer boot parameters in the first poll request after start up. */
	m_transfer_boot_params = false;
	/* Disable the ACK by default, it is enabled later if the server wants it. */
	m_ACK_bSendPeriodicLogs = false;
	m_ACK_bSendCoreDumps = false;
	m_ACK_xLogConfig = false;

	if (atomic_cas(&m_new_mdm_fw_update_state, MDM_FW_DOWNLOAD_COMPLETE, 0)) {
		struct messaging_mdm_fw_event *mdm_fw_ver = new_messaging_mdm_fw_event();
		mdm_fw_ver->buf = NULL;
		mdm_fw_ver->len = 0;
		EVENT_SUBMIT(mdm_fw_ver);
	}

	atomic_cas(&m_new_mdm_fw_update_state, INSTALLATION_COMPLETE, 0);

	PollMessageResponse *pResp = &proto->m.poll_message_resp;

	if (pResp->has_bSendPeriodicLogs) {
		m_ACK_bSendPeriodicLogs = true;
		m_upload_periodic_logs = (uint8_t)pResp->bSendPeriodicLogs;
		err = stg_config_u8_write(STG_U8_M_UPLOAD_PERIODIC_LOGS,
					  (uint8_t)pResp->bSendPeriodicLogs);
		if (err != 0) {
			NCLOG_WRN(MESSAGING_MODULE, TRice( iD( 5479), "wrn: Failed writing m_upload_periodic_logs to ext flash %d\n", err));
		}
	}
	if (pResp->has_bSendCoreDumps) {
		m_ACK_bSendCoreDumps = true;
		m_upload_core_dumps = (uint8_t)pResp->bSendCoreDumps;
		err = stg_config_u8_write(STG_U8_M_UPLOAD_CORE_DUMPS,
					  (uint8_t)pResp->bSendCoreDumps);
		if (err != 0) {
			NCLOG_WRN(MESSAGING_MODULE, TRice( iD( 1658), "wrn: Failed writing m_upload_core_dumps to ext flash %d\n", err));
		}
	}
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
	/* 	If we are asked to, reboot */
	if (pResp->has_bReboot && pResp->bReboot) {
		struct pwr_reboot_event *r_ev = new_pwr_reboot_event();
		r_ev->reason = REBOOT_SERVER_RESET;
		EVENT_SUBMIT(r_ev);
	}
	/* TODO: set activation mode to (pResp->eActivationMode); */

	if (pResp->has_bUseServerTime && pResp->bUseServerTime) {
		NCLOG_INF(MESSAGING_MODULE, TRice0( iD( 7873),"inf: Set date and time from server\n"));
		time_t gm_time = (time_t)proto->header.ulUnixTimestamp;
		struct tm *tm_time = gmtime(&gm_time);
		/* Update date_time library which storage uses for ANO data. */
		err = date_time_set(tm_time);
		if (err) {
			NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 4206),"err: Error updating time from server %d\n", err));
		} else {
			/** @note This prints UTC. */
			NCLOG_INF(MESSAGING_MODULE, TRice0( iD( 3653),"inf: Set timestamp to date_time library from modem: dynamic_string\n"));
			atomic_set(&server_timestamp_sec,
				   (atomic_val_t)(int32_t)(k_uptime_get_32() / 1000));
		}
	}
	if (pResp->has_usPollConnectIntervalSec) {
		/* Update poll request interval if not equal to interval requested by server */
		if (atomic_get(&poll_period_seconds) != pResp->usPollConnectIntervalSec) {
			atomic_set(&poll_period_seconds, pResp->usPollConnectIntervalSec);

			err = k_work_reschedule_for_queue(
				&message_q, &modem_poll_work,
				K_SECONDS(atomic_get(&poll_period_seconds)));
			if (err < 0) {
				NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 3998),"err: Failed to schedule work\n"));
			}

			NCLOG_INF(MESSAGING_MODULE, TRice( iD( 3246),"inf: Poll period of %d seconds will be used\n", atomic_get(&poll_period_seconds)));
			uint32_t wdt_module_ts =
				atomic_get(&poll_period_seconds) * CONFIG_WDT_KEEP_ALIVE_NUM_POLLS;
			/* Setup the KEEP_ALIVE watchdog to be smallest of WDT_KEEP_ALIVE_NUM_POLLS and CONFIG_WDT_KEEP_MAX_TIME_SECONDS */
			if (CONFIG_WDT_KEEP_ALIVE_NUM_POLLS * atomic_get(&poll_period_seconds) >
			    CONFIG_WDT_KEEP_MAX_TIME_SECONDS) {
				wdt_module_ts = CONFIG_WDT_KEEP_MAX_TIME_SECONDS;
			}

			nofence_wdt_module_register(WDT_MODULE_KEEP_ALIVE,
						    REBOOT_WDT_RESET_KEEP_ALIVE, wdt_module_ts);
		}
	}
	m_confirm_acc_limits = false;
	if (pResp->has_usAccSigmaSleepLimit) {
		m_confirm_acc_limits = true;
		err = stg_config_u16_write(STG_U16_ACC_SIGMA_SLEEP_LIMIT,
					   pResp->usAccSigmaSleepLimit);
		if (err != 0) {
			NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 7446),"err: Error updating sleep sigma to ext flash %d\n", err));
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
			NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 3016),"err: Error updating no activity sigma to ext flash %d\n", err));
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
			NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 1517),"err: Error updating off animal sigma to ext flash %d\n", err));
		}

		struct acc_sigma_event *sigma_ev = new_acc_sigma_event();
		sigma_ev->type = OFF_ANIMAL_SIGMA;
		sigma_ev->param.off_animal_value = pResp->usOffAnimalTimeLimitSec;
		EVENT_SUBMIT(sigma_ev);
	}
	m_confirm_urat_arg = false;
	if (pResp->has_xModemUratArg) {
		m_confirm_urat_arg = true;
		char buf[STG_CONFIG_URAT_ARG_BUF_SIZE];
		memset(buf, 0, sizeof(buf));
		strncpy(buf, pResp->xModemUratArg, sizeof(buf) - 1);
		err = stg_config_str_write(STG_STR_MODEM_URAT_ARG, buf, sizeof(buf) - 1);
		if (err != 0) {
			NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 7821),"err: Error storing URAT to NVS (%d)\n", err));
		} else {
			struct urat_args_received_event *urat_ev = new_urat_args_received_event();
			EVENT_SUBMIT(urat_ev);
		}
	}

	m_confirm_ble_key = false;
	if (pResp->has_rgubcBleKey) {
		m_confirm_ble_key = true;
		NCLOG_INF(MESSAGING_MODULE, TRice( iD( 5027),"inf: Received a ble_sec_key of size %d\n", pResp->rgubcBleKey.size));

		uint8_t current_ble_sec_key[STG_CONFIG_BLE_SEC_KEY_LEN];
		uint8_t key_length = 0;
		stg_config_blob_read(STG_BLOB_BLE_KEY, current_ble_sec_key, &key_length);
		int ret = memcmp(pResp->rgubcBleKey.bytes, current_ble_sec_key,
				 pResp->rgubcBleKey.size);
		if (ret != 0) {
			NCLOG_INF(MESSAGING_MODULE, TRice0( iD( 7935),"inf: New ble sec key is different. Will update ext flash\n"));
			ret = stg_config_blob_write(STG_BLOB_BLE_KEY, pResp->rgubcBleKey.bytes,
						    pResp->rgubcBleKey.size);
			if (ret < 0) {
				NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 4824),"err: Failed to write ble sec key to ext flash %d\n", ret));
			}
		}
	}
	if (pResp->ulFenceDefVersion != current_state.fence_version) {
		NCLOG_INF(MESSAGING_MODULE, TRice( iD( 4529),"inf: Requesting frame 0 for fence version %i.\n", pResp->ulFenceDefVersion));

		/* Submit a fence frame message request */
		m_fence_update_req.version = pResp->ulFenceDefVersion;
		m_fence_update_req.request_frame = 0;
		err = k_work_reschedule_for_queue(&message_q, &m_fence_update_req.work, K_NO_WAIT);
		if (err < 0) {
			NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 2283),"err: Failed to schedule work\n"));
		}
	}
#ifdef CONFIG_STG_CONFIG_DEBUG_SEND_WRITE_ERRORS
	/* If we sent NVS errors and there are no new ones, clear errors.
	 * Otherwise, keep sending
	 */
	if (m_sent_nvs_errors > 0) {
		if (g_nvs_write_errors > m_sent_nvs_errors) {
			g_nvs_write_errors -= m_sent_nvs_errors;
		} else {
			g_nvs_write_errors = 0;
		}
	}

#endif

	if (pResp->has_versionInfo) {
		if (process_upgrade_request(&pResp->versionInfo) == 0) {
			return;
		}
	}

	if (pResp->has_xModemFwFileName) {
		strncpy(mdm_fw_file_name, pResp->xModemFwFileName,
			sizeof(pResp->xModemFwFileName) - 1);
		NCLOG_INF(MESSAGING_MODULE, TRice0( iD( 2794),"inf: dynamic_string\n"));
		struct messaging_mdm_fw_event *mdm_fw_ver = new_messaging_mdm_fw_event();
		mdm_fw_ver->buf = mdm_fw_file_name;
		mdm_fw_ver->len = sizeof(pResp->xModemFwFileName);
		EVENT_SUBMIT(mdm_fw_ver);
	}

	if (pResp->has_bUseUbloxAno) {
		/* kick off ANO-download if needed. In effect, each poll message response
		 * will check if we should start download new AssistNowOffline data*
		 */
		check_kickoff_ano_download_start();
	}

	if (pResp->has_xLogConfig) {
		m_ACK_xLogConfig = true;
		for (int i = 0; i < pResp->xLogConfig.xModule_count; i++) {
			int ret = nclog_set_level((eNCLOG_MODULE)pResp->xLogConfig.xModule[i].xName,
						  (eNCLOG_LVL)pResp->xLogConfig.xModule[i].xLevel);
			if (ret != 0) {
				LOG_ERR("nclog_set_level returned -EINVAL");
			}
		}
	}

	return;
}

/** @todo : This is code duplication, create a utility parsing host and port from NVS */
static int get_and_parse_server_ip_address(char *buf, size_t size)
{
	uint8_t port_length = 0;
	int ret = stg_config_str_read(STG_STR_HOST_PORT, buf, size, &port_length);
	if (ret != 0) {
		NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 4124),"err: Failed to read host address from ext flash\n"));
		return ret;
	}

	/* Check if port is present */
	if (strchr(buf, ':') == NULL) {
		NCLOG_ERR(MESSAGING_MODULE, TRice0( iD( 6136),"err: Server address read from flash does not contain port\n"));
		return -EINVAL;
	}

	return 0;
}

/**
 * @brief Process an incoming firmware upgrade request, and starts the firmware download if a new
 * version is available on the server.
 * @param fw_ver_from_server The firmware version available on the server.
 */
static int process_upgrade_request(VersionInfoFW *fw_ver_from_server)
{
	if (fw_ver_from_server->has_ulApplicationVersion &&
	    fw_ver_from_server->ulApplicationVersion != NF_X25_VERSION_NUMBER &&
	    block_fota_request == false) {
		NCLOG_INF(MESSAGING_MODULE, TRice( iD( 7860),"inf: Received new app version from server %i\n", fw_ver_from_server->ulApplicationVersion));
		if (!reboot_scheduled) {
			m_fota_attempts++;
			struct start_fota_event *ev = new_start_fota_event();
			if (get_and_parse_server_ip_address(ev->host, sizeof(ev->host)) == 0) {
				ev->override_default_host = true;
			} else {
				NCLOG_WRN(MESSAGING_MODULE, TRice0( iD( 6014),"wrn: Cannot parse server address\n"));
				ev->override_default_host = false;
			}
			ev->serial_id = serial_id;
			ev->version = fw_ver_from_server->ulApplicationVersion;
			EVENT_SUBMIT(ev);
			return 0;
		}
	}
	return -1;
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
			NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 7108),"err: Unexpected frame count for pasture header. %d\n", -EIO));
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
				NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 7694),"err: Error writing keep mode to storage. %d\n", err));
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

	NCLOG_INF(MESSAGING_MODULE, TRice( iD( 7574),"inf: Cached fence frame %i successfully.\n", frame));
	if (frame == fenceResp->ucTotalFrames - 1) {
		/* Validate pasture. */
		if (cached_fences_counter != pasture_temp.m.ul_total_fences) {
			NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 6059),"err: Cached %i frames, but expected %i.\n", cached_fences_counter, pasture_temp.m.ul_total_fences));
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
			NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 6979),"err: CRC was not correct for new pasture. %d\n", -EIO));
			return 0;
		}

		NCLOG_INF(MESSAGING_MODULE, TRice0( iD( 4881),"inf: Validated CRC for pasture and will write it to flash.\n"));
		err = stg_write_pasture_data((uint8_t *)&pasture_temp, sizeof(pasture_temp));
		if (err) {
			return err;
		}
		return DOWNLOAD_COMPLETE;
	}

	return frame;
}

/**
 * @brief Process an incomming ANO message, chopping it up in UBX-MGA_ANO messages and
 * saving it to flash.
 * @param anoResp The ANO protobuf message
 * @retval < 0 Error occurred. The value gives the detailed reason
 * @retval > 0 Needs to download more ANO packages
 * @retval 0 ANO data successfully saved and we don't need more
 *
 */
static int process_ano_msg(UbxAnoReply *anoResp)
{
	int err;
	UBX_MGA_ANO_RAW_t *temp = NULL;
	ano_rec_t ano_rec;
	uint8_t n_mga_ano = anoResp->rgucBuf.size / sizeof(UBX_MGA_ANO_RAW_t);
	if (n_mga_ano == 0) {
		return -ENODATA;
	}
	for (int i = 0; i < n_mga_ano; i++) {
		memset(&ano_rec, 0, sizeof(ano_rec));
		temp = (UBX_MGA_ANO_RAW_t *)(anoResp->rgucBuf.bytes +
					     sizeof(UBX_MGA_ANO_RAW_t) * i);
		/*
		 * usAnoId identifies the day of the year (1-366) when the packet was generated.
		 * And we would like to use the closest day to the current date when
		 * feeding the GNSS receiver.
		 */

		ano_rec.ano_id = anoResp->usAnoId;
		ano_rec.sequence_id = anoResp->usStartAno + i;

		memcpy(&ano_rec.raw_ano, temp, sizeof(UBX_MGA_ANO_RAW_t));
		err = stg_write_ano_data(&ano_rec);

		if (err) {
			NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 1845),"err: Error writing ano frame to storage controller %d\n", err));
			return err;
		}
	}

	uint32_t ano_time_md = ano_date_to_unixtime_midday(temp->mga_ano.year, temp->mga_ano.month,
							   temp->mga_ano.day);

	int64_t current_time_ms = 0;

	err = date_time_now(&current_time_ms);
	if (err) {
		NCLOG_ERR(MESSAGING_MODULE, TRice( iD( 6411),"err: Error fetching date time %d\n", err));
		return err;
	}
	/* Use the last record in the ANO message to stamp NVS data */
	err = stg_config_u16_write(STG_U16_ANO_ID, anoResp->usAnoId);
	if (err) {
		return err;
	}
	err = stg_config_u16_write(STG_U16_ANO_START_ID, anoResp->usStartAno + n_mga_ano);
	if (err) {
		return err;
	}
	err = stg_config_u32_write(STG_U32_ANO_TIMESTAMP, ano_time_md);
	if (err) {
		return err;
	}
	if (ano_time_md > (current_time_ms / 1000) + TWO_AND_A_HALF_DAYS_SEC) {
		err = stg_config_u16_write(STG_U16_LAST_GOOD_ANO_ID, anoResp->usAnoId);
		if (err) {
			return err;
		}
		/* Fire an event so that GNSS can be updated */
		struct ano_ready_event *ano_ready = new_ano_ready_event();
		EVENT_SUBMIT(ano_ready);
	}

	return 0;
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
		.xNavPlExtra = { .ucTmirCoeff = gpsLastFix->pl.tmirCoeff,
				 .ucTmirExp = gpsLastFix->pl.tmirExp,
				 .ucPlPosValid = gpsLastFix->pl.plPosValid,
				 .ucPlPosFrame = gpsLastFix->pl.plPosFrame,
				 .ulPlPos1 = gpsLastFix->pl.plPos1,
				 .ulPlPos2 = gpsLastFix->pl.plPos2,
				 .ulPlPos3 = gpsLastFix->pl.plPos3 }

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
 * @brief Sets the initial collar state flag indicating that the module has recieved a collar state.
 * 		  The initial collar state flags are only set once for system startup sequence.
 * @param[in] uint8_t A collar state flag indicator (see collar_state_flags).
 */
static inline void set_initial_collar_state_flag(uint8_t aFlag)
{
	if (aFlag >= COLLAR_STATE_FLAG_CNT) {
		return;
	}
	m_initial_collar_state_flags |= (1 << aFlag);
}
/**
 * @brief Checks whether the module has recieved all initial collar states (see collar_state_flags).
 * @return Return true if all initial collar states has been received, otherwise false.
 */
static inline bool has_initial_collar_states()
{
	return (m_initial_collar_state_flags ==
				((1 << COLLAR_MODE_FLAG) | (1 << COLLAR_STATUS_FLAG) |
				 (1 << FENCE_STATUS_FLAG) | (1 << BATTERY_LVL_FLAG)) ?
			      true :
			      false);
}
