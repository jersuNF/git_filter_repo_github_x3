/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include <stdlib.h>
#include "messaging_module_events.h"
#include "messaging.h"
#include <logging/log.h>
#include "ble_ctrl_event.h"
#include "ble_data_event.h"
#include "lte_proto_event.h"

#include "cellular_controller_events.h"
#include "gps_controller_events.h"
#include "request_events.h"
#include "nf_version.h"
#include "collar_protocol.h"
#include "UBX.h"
#include "unixTime.h"
#include "error_event.h"
#include "helpers.h"

#include "storage_event.h"

#include "storage.h"

#include "pasture_structure.h"
#include "fw_upgrade_events.h"

#define DOWNLOAD_COMPLETE 255
#define GPS_UBX_NAV_PVT_VALID_HEADVEH_MASK 0x20
#define SECONDS_IN_THREE_DAYS 259200

#define BYTESWAP16(x) (((x) << 8) | ((x) >> 8))

uint32_t time_from_server;

K_SEM_DEFINE(cache_lock_sem, 1, 1);
K_SEM_DEFINE(send_ack_sem, 0, 1);

collar_state_struct_t current_state;
gps_last_fix_struct_t cached_fix;

/* Cached fence for constructing the new fence from fence_download frames. */
static fence_t *new_fence_points = NULL;
static size_t new_fence_size;
static size_t curr_copied_coords = 0;

static uint32_t new_fence_in_progress;
static uint8_t expected_fframe, expected_ano_frame, new_ano_in_progress;
static bool first_frame, first_ano_frame;

void build_poll_request(NofenceMessage *);
int8_t request_fframe(uint32_t, uint8_t);
void fence_download(uint8_t);
int8_t request_ano_frame(uint16_t, uint16_t);
void ano_download(uint16_t, uint16_t);
void proto_InitHeader(NofenceMessage *);
void process_poll_response(NofenceMessage *);
void process_upgrade_request(VersionInfoFW *);
uint8_t process_fence_msg(FenceDefinitionResponse *);
uint8_t process_ano_msg(UbxAnoReply *);

int encode_and_send_message(NofenceMessage *);
int send_binary_message(uint8_t *, size_t);

void messaging_thread_fn(void);

_DatePos proto_getLastKnownDatePos(gps_last_fix_struct_t *);
bool proto_hasLastKnownDatePos(const gps_last_fix_struct_t *);
static uint32_t ano_date_to_unixtime_midday(uint8_t, uint8_t, uint8_t);
bool m_confirm_acc_limits, m_confirm_ble_key, m_transfer_boot_params;
bool send_out_ack, use_server_time;

K_MUTEX_DEFINE(send_binary_mutex);

#define MODULE messaging
LOG_MODULE_REGISTER(MODULE, CONFIG_MESSAGING_LOG_LEVEL);

/* 4 means 4-byte alignment. */
K_MSGQ_DEFINE(ble_ctrl_msgq, sizeof(struct ble_ctrl_event),
	      CONFIG_MSGQ_BLE_CTRL_SIZE, 4);
K_MSGQ_DEFINE(ble_data_msgq, sizeof(struct ble_data_event),
	      CONFIG_MSGQ_BLE_DATA_SIZE, 4);
K_MSGQ_DEFINE(lte_proto_msgq, sizeof(struct lte_proto_event),
	      CONFIG_MSGQ_LTE_PROTO_SIZE, 4);

#define NUM_MSGQ_EVENTS 3
struct k_poll_event msgq_events[NUM_MSGQ_EVENTS] = {
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
					K_POLL_MODE_NOTIFY_ONLY, &ble_ctrl_msgq,
					0),
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
					K_POLL_MODE_NOTIFY_ONLY, &ble_data_msgq,
					0),
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
					K_POLL_MODE_NOTIFY_ONLY,
					&lte_proto_msgq, 0),
};

static struct k_work_q send_q;
struct k_work_delayable modem_poll_work;
struct k_work_delayable log_work;

atomic_t poll_period_minutes = ATOMIC_INIT(5);
atomic_t log_period_minutes = ATOMIC_INIT(30);

K_THREAD_DEFINE(messaging_thread, CONFIG_MESSAGING_THREAD_SIZE,
		messaging_thread_fn, NULL, NULL, NULL,
		CONFIG_MESSAGING_THREAD_PRIORITY, 0, 0);

K_KERNEL_STACK_DEFINE(messaging_send_thread, CONFIG_MESSAGING_SEND_THREAD_SIZE);

void cleanup_cached_fence_resources(void)
{
	/* Free and cleanup the resources of the cached fence
	 * as we have now verified that AMC has retrieved 
	 * the new fence and started using it.
	 */
	k_free(new_fence_points);
	new_fence_points = NULL;
	new_fence_size = 0;
	curr_copied_coords = 0;
}

void build_log_message()
{
	/* Fill in NofenceMessage and encode it, then store on fcb. Do it twice,
	 * once for seq message 1 and once for seq message 2. As such;

	 * NofenceMessage seq_1;
	 * NofenceMessage seq_2;
	 * 
	 * seq_1.which_m = NofenceMessage_seq_msg_tag;
	 * seq_2.which_m = NofenceMessage_seq_msg_2_tag;
	 * ...
	 * ...
	 * 
	 * collar_protocol_encode(seq_1, dst1, dst_max_size1, dst_size1);
	 * collar_protocol_encode(seq_2, dst2, dst_max_size2, dst_size2);
	 * 
	 * stg_write_log_data(dst1, dst_size1);
	 * stg_write_log_data(dst2, dst_size2);
	 * 
	 * DONE.
	 */
	return;
}

int read_log_data_cb(uint8_t *data, size_t len)
{
	uint8_t *bytes = k_malloc(len + 2);
	memcpy(&bytes[2], data, len);

	int err = send_binary_message(bytes, len + 2);
	if (err) {
		LOG_ERR("Error sending binary message for log data %i", err);
	}
	k_free(bytes);
	return err;
}

/**
 * @brief Build, send a log request, and reschedule after
 *        "log_period_minutes" minutes.
 */
void log_data_periodic_fn()
{
	/* Construct log data and write to storage controller. */
	build_log_message();

	/* Read and send out all the log data if any. */
	int err = stg_read_log_data(read_log_data_cb, 0);
	if (err && err != -ENODATA) {
		LOG_ERR("Error reading all sequence messages from storage %i",
			err);
	} else if (err == -ENODATA) {
		LOG_INF("No log data available on flash for sending.");
	}

	/* If all entries has been consumed, empty storage. */
	if (stg_log_pointing_to_last()) {
		err = stg_clear_partition(STG_PARTITION_LOG);
		if (err) {
			LOG_ERR("Error clearing FCB storage for LOG %i", err);
		}
		LOG_INF("Emptied LOG partition data as we have read everything.");
	}

	/* Reschedule. */
	k_work_reschedule_for_queue(&send_q, &log_work,
				    K_MINUTES(atomic_get(&log_period_minutes)));
}

/**
 * @brief Build, send a poll request, and reschedule after
 * "poll_period_minutes".
 */
void modem_poll_work_fn()
{
	/* Add logic for the periodic protobuf modem poller. */
	LOG_INF("Starting periodic poll work fn!\n");
	NofenceMessage new_poll_msg;

	if (!k_sem_take(&cache_lock_sem, K_SECONDS(1))) {
		LOG_DBG("Building poll request!\n");
		build_poll_request(&new_poll_msg);
		k_sem_give(&cache_lock_sem);
	}

	LOG_INF("Messaging - Sending new message!\n");
	encode_and_send_message(&new_poll_msg);

	k_work_reschedule_for_queue(
		&send_q, &modem_poll_work,
		K_MINUTES(atomic_get(&poll_period_minutes)));
}

/**
 * @brief Main event handler function. 
 * 
 * @param[in] eh Event_header for the if-chain to 
 *               use to recognize which event triggered.
 * 
 * @return True or false based on if we want to consume the event or not.
 */
static bool event_handler(const struct event_header *eh)
{
	if (is_ble_ctrl_event(eh)) {
		struct ble_ctrl_event *ev = cast_ble_ctrl_event(eh);
		while (k_msgq_put(&ble_ctrl_msgq, ev, K_NO_WAIT) != 0) {
			/* Message queue is full: purge old data & try again */
			k_msgq_purge(&ble_ctrl_msgq);
		}
		return false;
	}
	if (is_ble_data_event(eh)) {
		struct ble_data_event *ev = cast_ble_data_event(eh);
		while (k_msgq_put(&ble_data_msgq, ev, K_NO_WAIT) != 0) {
			k_msgq_purge(&ble_data_msgq);
		}
		return false;
	}
	if (is_cellular_proto_in_event(eh)) {
		struct cellular_proto_in_event *ev =
			cast_cellular_proto_in_event(eh);
		while (k_msgq_put(&lte_proto_msgq, ev, K_NO_WAIT) != 0) {
			k_msgq_purge(&lte_proto_msgq);
		}
		return false;
	}
	if (is_update_collar_mode(eh)) {
		struct update_collar_mode *ev = cast_update_collar_mode(eh);
		current_state.collar_mode = ev->collar_mode;
		return false;
	}
	if (is_update_collar_status(eh)) {
		struct update_collar_status *ev = cast_update_collar_status(eh);
		current_state.collar_status = ev->collar_status;
		return false;
	}
	if (is_update_fence_status(eh)) {
		struct update_fence_status *ev = cast_update_fence_status(eh);
		current_state.fence_status = ev->fence_status;
		return false;
	}
	if (is_update_fence_version(eh)) {
		struct update_fence_version *ev = cast_update_fence_version(eh);
		current_state.fence_version = ev->fence_version;

		/* Notify server as soon as the new fence is activated. */
		modem_poll_work_fn();
		return false;
	}
	if (is_update_flash_erase(eh)) {
		current_state.flash_erase_count++;
		return false;
	}
	if (is_update_zap_count(eh)) {
		struct update_zap_count *ev = cast_update_zap_count(eh);
		current_state.zap_count = ev->count;
		return false;
	}
	if (is_cellular_ack_event(eh)) {
		k_sem_give(&send_ack_sem);
		return false;
	}
	if (is_new_gps_fix(eh)) {
		struct new_gps_fix *ev = cast_new_gps_fix(eh);
		if (!k_sem_take(&cache_lock_sem, K_SECONDS(0.1))) {
			cached_fix = ev->fix;
			k_sem_give(&cache_lock_sem);
		}
		return false;
	}
	if (is_request_ano_event(eh)) {
		request_ano_frame(54, 0);
		//request frame 0
		first_ano_frame = true;
		LOG_WRN("Requesting ano data!\n");
		int ret = request_ano_frame(0, 0);
		if (ret == 0) {
			first_ano_frame = false;
		}
		return false;
	}

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

int crc_check_msg_cached_fence()
{
	//uint16_t crc = 0xFFFF;
	//uint16_t eepUint16;
	//uint16_t receivedCRC = new_fence_points->
	//
	//if (receivedCRC == EMPTY_FENCE_CRC) {
	//	//we received old fence definition without crc, or we have just migrated and the
	//	// current fenceDef will not be validated until a new one is downloaded
	//	return true;
	//}
	//
	//uint8_t fenceCount = EEPROM_GetNumberOfFences();
	//uint32_t eepUint32 = EEPROM_GetFenceConfigCrc();
	//FenceHeader header = { 0 };
	//
	//crc = nf_crc16_uint32(eepUint32, &crc);
	//eepUint32 = EEPROM_GetOriginLon();
	//crc = nf_crc16_uint32(eepUint32, &crc);
	//eepUint32 = EEPROM_GetOriginLat();
	//crc = nf_crc16_uint32(eepUint32, &crc);
	//eepUint16 = EEPROM_GetK_LON();
	//crc = nf_crc16_uint16(eepUint16, &crc);
	//eepUint16 = EEPROM_GetK_LAT();
	//crc = nf_crc16_uint16(eepUint16, &crc);
	//
	//for (uint8_t fenceNo = 0; fenceNo < fenceCount; fenceNo++) {
	//	EEPROM_ReadFenceHeader(fenceNo, &header);
	//	for (uint8_t i = 0; i < header.nPoints; i++) {
	//		eepUint32 = eeprom_read_dword((void *)(header.pC + i));
	//		crc = nf_crc16_uint32(eepUint32, &crc);
	//	}
	//}
	//
	//return crc == receivedCRC;
	//
	return 0;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, ble_ctrl_event);
EVENT_SUBSCRIBE(MODULE, ble_data_event);
EVENT_SUBSCRIBE(MODULE, lte_proto_event);
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

static inline void process_ble_ctrl_event(void)
{
	struct ble_ctrl_event ev;

	int err = k_msgq_get(&ble_ctrl_msgq, &ev, K_NO_WAIT);
	if (err) {
		LOG_ERR("Error getting ble_ctrl_event: %d\n", err);
		return;
	}
}

static inline void process_ble_data_event(void)
{
	struct ble_data_event ev;

	int err = k_msgq_get(&ble_data_msgq, &ev, K_NO_WAIT);
	if (err) {
		LOG_ERR("Error getting ble_data_event: %d\n", err);
		return;
	}
}

static void process_lte_proto_event(void)
{
	struct cellular_proto_in_event ev;

	int err = k_msgq_get(&lte_proto_msgq, &ev, K_NO_WAIT);
	if (err) {
		LOG_ERR("Error getting lte_proto_event: %d\n", err);
		return;
	}

	NofenceMessage proto;
	err = collar_protocol_decode(ev.buf + 2, ev.len - 2, &proto);
	LOG_WRN("Number of received bytes = %d\n", ev.len);

	char *buf = ev.buf;
	for (int i = 0; i < ev.len; i++) {
		printk("\\x%02x", *buf);
		buf++;
	}
	if (err) {
		LOG_ERR("Error decoding protobuf message. %i", err);
		return;
	}
	struct messaging_ack_event *ack = new_messaging_ack_event();
	EVENT_SUBMIT(ack);
	/* process poll response */
	if (proto.which_m == NofenceMessage_poll_message_resp_tag) {
		process_poll_response(&proto);
		return;
	} else if (proto.which_m == NofenceMessage_fence_definition_resp_tag) {
		uint8_t new_fframe =
			process_fence_msg(&proto.m.fence_definition_resp);
		fence_download(new_fframe);
		return;
	} else if (proto.which_m == NofenceMessage_ubx_ano_reply_tag) {
		uint16_t new_ano_frame =
			process_ano_msg(&proto.m.ubx_ano_reply);
		ano_download(proto.m.ubx_ano_reply.usAnoId, new_ano_frame);
		return;
	} else {
		return;
	}
}
void messaging_thread_fn()
{
	while (true) {
		int rc = k_poll(msgq_events, NUM_MSGQ_EVENTS, K_FOREVER);
		if (rc == 0) {
			while (k_msgq_num_used_get(&ble_ctrl_msgq) > 0) {
				process_ble_ctrl_event();
			}
			while (k_msgq_num_used_get(&ble_data_msgq) > 0) {
				process_ble_data_event();
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

void messaging_module_init(void)
{
	LOG_INF("Initializing messaging module!\n");
	k_work_queue_init(&send_q);
	k_work_queue_start(&send_q, messaging_send_thread,
			   K_THREAD_STACK_SIZEOF(messaging_send_thread),
			   CONFIG_MESSAGING_SEND_THREAD_PRIORITY, NULL);

	k_work_init_delayable(&modem_poll_work, modem_poll_work_fn);
	k_work_init_delayable(&log_work, log_data_periodic_fn);

	k_work_schedule_for_queue(&send_q, &modem_poll_work, K_NO_WAIT);
	k_work_schedule_for_queue(&send_q, &log_work, K_NO_WAIT);
}

void build_poll_request(NofenceMessage *poll_req)
{
	proto_InitHeader(poll_req); /* fill up message header. */
	poll_req->which_m = NofenceMessage_poll_message_req_tag;
	poll_req->m.poll_message_req.datePos =
		proto_getLastKnownDatePos(&cached_fix);
	poll_req->m.poll_message_req.has_datePos =
		proto_hasLastKnownDatePos(&cached_fix);
	poll_req->m.poll_message_req.eMode = current_state.collar_mode;
	poll_req->m.poll_message_req.usZapCount = current_state.zap_count;
	poll_req->m.poll_message_req.eCollarStatus =
		current_state.collar_status;
	poll_req->m.poll_message_req.eFenceStatus = current_state.fence_status;
	poll_req->m.poll_message_req.ulFenceDefVersion =
		current_state.fence_version;
	poll_req->m.poll_message_req.usBatteryVoltage = 378; /* TODO: get
 * value from battery voltage event.*/
	poll_req->m.poll_message_req.has_ucMCUSR = 0;
	poll_req->m.poll_message_req.ucMCUSR = 0;
	/* TODO: get gsm info from modem driver */
	//	const _GSM_INFO *p_gsm_info = bgs_get_gsm_info();
	//	poll_req.m.poll_message_req.xGsmInfo = *p_gsm_info;
	//	poll_req->m.poll_message_req.has_xGsmInfo = false;

	if (current_state.flash_erase_count) {
		// m_flash_erase_count is reset when we receive a poll reply
		poll_req->m.poll_message_req.has_usFlashEraseCount = true;
		poll_req->m.poll_message_req.usFlashEraseCount =
			current_state.flash_erase_count;
	}
	if (m_confirm_acc_limits) {
		//		poll_req.m.poll_message_req.has_usAccSigmaSleepLimit = true;
		//		poll_req.m.poll_message_req.usAccSigmaSleepLimit =
		//			EEPROM_GetAccSigmaSleepLimit();
		//		poll_req.m.poll_message_req.has_usAccSigmaNoActivityLimit = true;
		//		poll_req.m.poll_message_req.usAccSigmaNoActivityLimit =
		//			EEPROM_GetAccSigmaNoActivityLimit();
		//		poll_req.m.poll_message_req.has_usOffAnimalTimeLimitSec = true;
		//		poll_req.m.poll_message_req.usOffAnimalTimeLimitSec =
		//			EEPROM_GetOffAnimalTimeLimitSec();
	}
	if (m_confirm_ble_key) {
		//		poll_req.m.poll_message_req.has_rgubcBleKey = true;
		//		poll_req.m.poll_message_req.rgubcBleKey.size = EEP_BLE_SEC_KEY_LEN;
		//		EEPROM_ReadBleSecKey(poll_req.m.poll_message_req.rgubcBleKey.bytes,
		//				     EEP_BLE_SEC_KEY_LEN);
	}
	poll_req->m.poll_message_req.usGnssOnFixAgeSec = 123;
	poll_req->m.poll_message_req.usGnssTTFFSec = 12;

	if (m_transfer_boot_params) {
		//		poll_req.m.poll_message_req.has_versionInfo = true;
		//		uint16_t xbootVersion;
		//		if (xboot_get_version(&xbootVersion) == XB_SUCCESS) {
		//			poll_req.m.poll_message_req.versionInfo
		//				.usATmegaBootloaderVersion =
		//				xbootVersion;
		//			poll_req.m.poll_message_req.versionInfo
		//				.has_usATmegaBootloaderVersion = true;
		//		}
		//		poll_req.m.poll_message_req.has_versionInfoHW = true;
		//		poll_req.m.poll_message_req.versionInfoHW.ucPCB_RF_Version =
		//			EEPROM_GetHwVersion();
		//		poll_req.m.poll_message_req.versionInfoHW.usPCB_Product_Type =
		//			(uint8_t)EEPROM_GetProductType();
		//		poll_req.m.poll_message_req.has_xSimCardId = true;
		//		memcpy(poll_req.m.poll_message_req.xSimCardId, BGS_SCID(),
		//		       sizeof(poll_req.m.poll_message_req.xSimCardId));
		//		poll_req.m.poll_message_req.versionInfo.has_ulATmegaVersion =
		//			true;
		//		poll_req.m.poll_message_req.versionInfo.ulATmegaVersion =
		//			NF_X25_VERSION_NUMBER;
	}
}

int8_t request_fframe(uint32_t version, uint8_t frame)
{
	NofenceMessage fence_req;
	proto_InitHeader(&fence_req); /* fill up message header. */
	fence_req.which_m = NofenceMessage_fence_definition_req_tag;
	fence_req.m.fence_definition_req.ulFenceDefVersion = version;
	fence_req.m.fence_definition_req.ucFrameNumber = frame;
	int ret = encode_and_send_message(&fence_req);
	if (ret) {
		LOG_WRN("Failed to send request for frame %d\n", frame);
		return -1;
	}
	return 0;
}

void fence_download(uint8_t new_fframe)
{
	if (new_fframe == 0 && first_frame) {
		first_frame = false;
	} else if (new_fframe == 0 && !first_frame) { //something went bad
		expected_fframe = 0;
		new_fence_in_progress = 0;
		return;
	}
	if (new_fframe >= 0) {
		if (new_fframe == DOWNLOAD_COMPLETE) {
			int err = crc_check_msg_cached_fence();
			if (err) {
				/* CRC error, try again? */
				expected_fframe = 0;
				new_fence_in_progress = 0;
				cleanup_cached_fence_resources();
				return;
			}

			/* Write to storage controller. */
			err = stg_write_pasture_data(
				(uint8_t *)new_fence_points, new_fence_size);

			if (err) {
				LOG_ERR("Error storing new fence to external flash.");
				return;
			}

			cleanup_cached_fence_resources();

			/* Notify AMC that a new fence is available. */
			struct new_fence_available *fence_ready =
				new_new_fence_available();
			fence_ready->fence_version = new_fence_in_progress;
			EVENT_SUBMIT(fence_ready);

			LOG_WRN("Fence %d download "
				"complete!\n",
				new_fence_in_progress);
			expected_fframe = 0;
			/* trigger ano download for the sake of testing only
			first_ano_frame = true;
			request_ano_frame(0, 0);
			 */
			return;
		}
		if (new_fframe == expected_fframe) {
			expected_fframe++;
			/* TODO: handle failure to send request!*/
			request_fframe(new_fence_in_progress, expected_fframe);
			LOG_WRN("Requesting frame %d of new fence: %d"
				".\n",
				expected_fframe, new_fence_in_progress);

		} else {
			expected_fframe = 0;
			new_fence_in_progress = 0;
			cleanup_cached_fence_resources();
			return;
		}
	}
}

int8_t request_ano_frame(uint16_t ano_id, uint16_t ano_start)
{
	NofenceMessage ano_req;
	proto_InitHeader(&ano_req); /* fill up message header. */
	ano_req.which_m = NofenceMessage_ubx_ano_req_tag;
	ano_req.m.ubx_ano_req.usAnoId = ano_id;
	ano_req.m.ubx_ano_req.usStartAno = ano_start;
	int ret = encode_and_send_message(&ano_req);
	if (ret) {
		LOG_WRN("Failed to send request for ano %d\n", ano_start);
		return -1;
	}
	return 0;
}

void ano_download(uint16_t ano_id, uint16_t new_ano_frame)
{
	LOG_WRN("ano_id = %d, ano_frame = %d\n", ano_id, new_ano_frame);
	if (first_ano_frame) {
		new_ano_in_progress = ano_id;
		expected_ano_frame = 0;
		first_ano_frame = false;
	} else if (new_ano_frame == 0 &&
		   !first_ano_frame) { //something went bad
		expected_ano_frame = 0;
		new_ano_in_progress = 0;
		return;
	}
	if (new_ano_frame >= 0) {
		if (new_ano_frame == DOWNLOAD_COMPLETE) {
			LOG_INF("ANO %d download "
				"complete!\n",
				new_ano_in_progress);
			return;
		}

		expected_ano_frame += new_ano_frame;
		LOG_INF("expected ano frame = %d\n", expected_ano_frame);
		/* TODO: handle failure to send request!*/
		int ret = request_ano_frame(ano_id, expected_ano_frame);

		LOG_INF("Requesting frame %d of new ano: %d"
			".\n",
			expected_ano_frame, new_ano_in_progress);

		if (ret != 0) {
			/* TODO: reset ANO state and retry later.*/
			expected_ano_frame = 0;
			new_ano_in_progress = 0;
		}
		return;
	}
}

void proto_InitHeader(NofenceMessage *msg)
{
	msg->header.ulId = 11500; //TODO: read from eeprom
	msg->header.ulVersion = NF_X25_VERSION_NUMBER;
	msg->header.has_ulVersion = true;
	if (use_server_time) {
		msg->header.ulUnixTimestamp = time_from_server;
	} else {
		msg->header.ulUnixTimestamp = cached_fix.unixTimestamp;
	}
}

/** @brief Sends a binary encoded message to the server through cellular
 *         controller.
 * @param data binary message. @note Assumes 2 first bytes are empty.
 * @param len length of the binary data including the 2 start bytes.
 * 
 * @return 0 on success, otherwise negative errno.
 */
int send_binary_message(uint8_t *data, size_t len)
{
	/* We can only send 1 message at a time, use mutex. */
	if (k_mutex_lock(&send_binary_mutex,
			 K_SECONDS(CONFIG_BINARY_SEND_TIMEOUT_SEC * 2)) == 0) {
		uint16_t byteswap_size = BYTESWAP16(len - 2);
		memcpy(&data[0], &byteswap_size, 2);

		struct messaging_proto_out_event *msg2send =
			new_messaging_proto_out_event();
		msg2send->buf = data;
		msg2send->len = len;
		EVENT_SUBMIT(msg2send);

		int err = k_sem_take(&send_ack_sem,
				     K_SECONDS(CONFIG_BINARY_SEND_TIMEOUT_SEC));
		if (err) {
			char *e_msg = "Timed out waiting for cellular ack!";
			nf_app_error(ERR_MESSAGING, -ETIMEDOUT, e_msg,
				     strlen(e_msg));
			k_mutex_unlock(&send_binary_mutex);
			return -ETIMEDOUT;
		}
		k_mutex_unlock(&send_binary_mutex);
		return 0;
	} else {
		return -ETIMEDOUT;
	}
}

int encode_and_send_message(NofenceMessage *msg_proto)
{
	uint8_t encoded_msg[NofenceMessage_size + 2];
	memset(encoded_msg, 0, sizeof(encoded_msg));
	size_t encoded_size = 0;

	LOG_INF("Start message encoding!, size: %d, version: %u\n",
		sizeof(msg_proto), msg_proto->header.ulVersion);
	int ret = collar_protocol_encode(msg_proto, &encoded_msg[2],
					 NofenceMessage_size, &encoded_size);
	if (ret) {
		LOG_ERR("Error encoding nofence message. %i", ret);
		return ret;
	}
	return send_binary_message(encoded_msg, encoded_size + 2);
}

void process_poll_response(NofenceMessage *proto)
{
	PollMessageResponse *pResp = &proto->m.poll_message_resp;
	if (pResp->has_xServerIp && strlen(pResp->xServerIp) > 0) {
		struct messaging_host_address_event *host_add_event =
			new_messaging_host_address_event();
		//		strcpy(host_add_event.address, pResp->xServerIp, sizeof(pResp->xServerIp));
		strncpy(host_add_event->address, pResp->xServerIp,
			sizeof(pResp->xServerIp));
		EVENT_SUBMIT(host_add_event); /*cellular controller writes it
 * to eeprom if it is different from the previously stored address.*/
	}
	if (pResp->has_bEraseFlash && pResp->bEraseFlash) {
		struct request_flash_erase_event *flash_erase_event =
			new_request_flash_erase_event();
		EVENT_SUBMIT(flash_erase_event);
	}
	// If we are asked to, reboot
	if (pResp->has_bReboot && pResp->bReboot) {
		/* TODO: publish reboot event to power manager */
	}
	/* TODO: set activation mode to (pResp->eActivationMode); */

	if (pResp->has_bUseUbloxAno) {
		/* TODO: publish enable ANO event to GPS controller */
	}
	if (pResp->has_bUseServerTime && pResp->bUseServerTime) {
		LOG_WRN("Server time will be used!\n");
		time_from_server = proto->header.ulUnixTimestamp;
		use_server_time = true;
	}
	if (pResp->has_usPollConnectIntervalSec) {
		atomic_set(&poll_period_minutes,
			   pResp->usPollConnectIntervalSec / 60);

		k_work_reschedule_for_queue(
			&send_q, &modem_poll_work,
			K_MINUTES(atomic_get(&poll_period_minutes)));
		LOG_WRN("Poll period of %d minutes will be used!\n",
			atomic_get(&poll_period_minutes));
	}
	if (pResp->has_usAccSigmaSleepLimit) {
		/* TODO: submit pResp->usAccSigmaSleepLimit to AMC module.
		 * AMC will store it in eeprom or external flash. */
	}
	if (pResp->has_usAccSigmaNoActivityLimit) {
		/* TODO: submit pResp->usAccSigmaNoActivityLimit to AMC
		 * module. */
	}
	if (pResp->has_usOffAnimalTimeLimitSec) {
		/* TODO: submit pResp->usOffAnimalTimeLimitSec to AMC. */
	}
	if (pResp->has_rgubcBleKey) {
		/* TODO: submit pResp->rgubcBleKey.bytes,pResp->rgubcBleKey
		 * .size to BLE controller. */
	}
	if (pResp->has_versionInfo) {
		process_upgrade_request(&pResp->versionInfo);
	}
	if (pResp->ulFenceDefVersion != current_state.fence_version &&
	    new_fence_in_progress != pResp->ulFenceDefVersion) {
		/* TODO: Download new fence and submit pResp->ulFenceDefVersion
		 * to AMC */
		//request frame 0
		first_frame = true;
		LOG_WRN("Requesting frame 0 of fence: %d!\n",
			pResp->ulFenceDefVersion);
		int ret = request_fframe(pResp->ulFenceDefVersion, 0);
		if (ret == 0) {
			first_frame = true;
			new_fence_in_progress = pResp->ulFenceDefVersion;
		}
	}
	return;
}

/* @brief: starts a firmware download if a new version exists on the server. */
void process_upgrade_request(VersionInfoFW *fw_ver_from_server)
{
	// compare versions and start update when needed.//
	uint32_t app_ver = fw_ver_from_server->ulNRF52AppVersion;

	if (app_ver > NF_X25_VERSION_NUMBER) {
		struct start_fota_event *ev = new_start_fota_event();
		ev->override_default_host = false;
		ev->version = app_ver;
		EVENT_SUBMIT(ev);
	} else {
		LOG_INF("Requested firmware version is \
			 same or older than current");
		return;
	}

	return;
}

uint8_t process_fence_msg(FenceDefinitionResponse *fenceResp)
{
	uint8_t frame = fenceResp->ucFrameNumber;
	LOG_INF("Received fence frame number %d\n", frame);

	if (new_fence_in_progress != fenceResp->ulFenceDefVersion) {
		/* Something went wrong, restart fence request. */
		return 0;
	}

	if (frame == 0) {
		if (new_fence_points != NULL || curr_copied_coords != 0 ||
		    new_fence_size != 0) {
			/* Something happend and the previous fence
			 * has not yet been consumed/freed correctly.
			 */
			return 0;
		}
		/* Setup the new fence. */
		new_fence_size =
			sizeof(fence_t) + (sizeof(fence_coordinate_t) *
					   fenceResp->m.xHeader.ulTotalFences);
		new_fence_points = (fence_t *)k_malloc(new_fence_size);

		new_fence_points->header.us_id = fenceResp->m.xFence.usId;
		new_fence_points->header.n_points =
			fenceResp->m.xHeader.ulTotalFences;
		new_fence_points->header.e_fence_type =
			fenceResp->m.xFence.eFenceType;

		LOG_INF("Total number of fence frames = %d",
			fenceResp->ucTotalFrames);
	}

	/* Copy up to 40 fence points from the frame to the cached fence. */
	memcpy(&new_fence_points->p_c[curr_copied_coords],
	       fenceResp->m.xFence.rgulPoints,
	       sizeof(fence_coordinate_t) *
		       fenceResp->m.xFence.rgulPoints_count);
	curr_copied_coords += fenceResp->m.xFence.rgulPoints_count;

	if (frame == fenceResp->ucTotalFrames - 1) {
		return DOWNLOAD_COMPLETE;
	}

	return frame;
}

uint8_t process_ano_msg(UbxAnoReply *anoResp)
{
	LOG_DBG("Ano response received!\n");
	uint8_t rec_ano_frames =
		anoResp->rgucBuf.size / sizeof(UBX_MGA_ANO_RAW_t);
	UBX_MGA_ANO_RAW_t *temp = NULL;
	temp = (UBX_MGA_ANO_RAW_t *)(anoResp->rgucBuf.bytes +
				     sizeof(UBX_MGA_ANO_RAW_t));

	uint32_t age = ano_date_to_unixtime_midday(
		temp->mga_ano.year, temp->mga_ano.month, temp->mga_ano.day);

	LOG_WRN("Relative age of received ANO frame = %d, %d \n", age,
		time_from_server);

	/* Write to storage controller's ANO WRITE partition. */
	int err = stg_write_ano_data((uint8_t *)&anoResp->rgucBuf,
				     anoResp->rgucBuf.size);

	if (err) {
		LOG_ERR("Error writing ano frame to storage controller %i",
			err);
	}

	if (age > time_from_server + SECONDS_IN_THREE_DAYS) {
		return DOWNLOAD_COMPLETE;
	}

	LOG_DBG("Number of received ano buffer = %d!\n", rec_ano_frames);
	return rec_ano_frames;
}

_DatePos proto_getLastKnownDatePos(gps_last_fix_struct_t *gpsLastFix)
{
	bool valid_headVeh = (bool)(gpsLastFix->pvt_flags &
				    GPS_UBX_NAV_PVT_VALID_HEADVEH_MASK);
	_DatePos a = {
		.lLat = gpsLastFix->Lat,
		.lLon = gpsLastFix->Lon,
		.usHorizontalAccDm = gpsLastFix->hAccDm,
		.ulUnixTimestamp = gpsLastFix->unixTimestamp,
		.has_sHeadVeh = valid_headVeh,
		.sHeadVeh = gpsLastFix->headVeh,
		.has_usHeadAcc = valid_headVeh,
		.usHeadAcc = gpsLastFix->headAcc,
		.has_ucNumSV = true,
		.ucNumSV = gpsLastFix->numSV,
		.has_usHDOP = true,
		.usHDOP = gpsLastFix->hdop,
		.has_lHeight = true,
		.lHeight = gpsLastFix->Height,
#if defined(HW_BAROMETER)
		.has_usHeight = true,
		.usHeight = gpsLastFix->baro_Height,
#endif
		.has_ucGpsMode = true,
		.ucGpsMode = gpsLastFix->gps_mode
	};
	return a;
}

bool proto_hasLastKnownDatePos(const gps_last_fix_struct_t *gps)
{
	return gps->unixTimestamp != 0;
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
