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

#define DOWNLOAD_COMPLETE 255
#define GPS_UBX_NAV_PVT_VALID_HEADVEH_MASK 0x20
#define SECONDS_IN_THREE_DAYS 259200

#define BYTESWAP16(x) (((x) << 8) | ((x) >> 8))

uint32_t time_from_server;

K_SEM_DEFINE(cache_lock_sem, 1, 1);

collar_state_struct_t current_state;
gps_last_fix_struct_t cached_fix;

static uint32_t new_fence_in_progress;
static uint8_t expected_fframe, expected_ano_frame, new_ano_in_progress;
static bool first_frame, first_ano_frame;

uint8_t poll_period_minutes = 5;
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
int send_message(NofenceMessage *);
void messaging_thread_fn(void);

_DatePos proto_getLastKnownDatePos(gps_last_fix_struct_t *);
bool proto_hasLastKnownDatePos(const gps_last_fix_struct_t *);
static uint32_t ano_date_to_unixtime_midday(uint8_t, uint8_t,
					    uint8_t);
bool m_confirm_acc_limits, m_confirm_ble_key, m_transfer_boot_params;
bool send_out_ack, use_server_time;

K_SEM_DEFINE(ble_ctrl_sem, 0, 1);
K_SEM_DEFINE(ble_data_sem, 0, 1);
K_SEM_DEFINE(lte_proto_sem, 0, 1);

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

static struct k_work_q poll_q;
struct k_work_delayable modem_poll_work;

K_THREAD_DEFINE(messaging_thread, CONFIG_MESSAGING_THREAD_SIZE,
		messaging_thread_fn, NULL, NULL, NULL,
		CONFIG_MESSAGING_THREAD_PRIORITY, 0, 0);

K_KERNEL_STACK_DEFINE(messaging_poll_thread, CONFIG_MESSAGING_POLL_THREAD_SIZE);

/**
 * @brief Build, send a poll request, and reschedule after
 * "poll_period_minutes".
 */
void modem_poll_work_fn()
{
	/* Add logic for the periodic protobuf modem poller. */
	LOG_WRN("Starting periodic poll work fn!\n");
	NofenceMessage new_poll_msg;
	memset(&new_poll_msg, 0, sizeof(new_poll_msg));
	if (!k_sem_take(&cache_lock_sem, K_SECONDS(1))) {
		LOG_DBG("Building poll request!\n");
		build_poll_request(&new_poll_msg);
		k_sem_give(&cache_lock_sem);
	}
	LOG_WRN("Messaging - Sending new message!\n");
	send_message(&new_poll_msg);
	k_work_reschedule_for_queue(&poll_q, &modem_poll_work,
				    K_SECONDS(poll_period_minutes * 60));
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
		struct cellular_proto_in_event *ev = cast_cellular_proto_in_event(eh);
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
		modem_poll_work_fn(); /* notify server as soon as the new
 * fence is activated.*/
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
		send_out_ack = true;
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
		if (ret == 0){
			first_ano_frame = false;
		}
		return false;
	}

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
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
	LOG_INF("Processed ble_ctrl_event.\n");
	k_sem_give(&ble_ctrl_sem);
}

static inline void process_ble_data_event(void)
{
	struct ble_data_event ev;

	int err = k_msgq_get(&ble_data_msgq, &ev, K_NO_WAIT);
	if (err) {
		LOG_ERR("Error getting ble_data_event: %d\n", err);
		return;
	}
	LOG_INF("Processed ble_data_event.\n");
	k_sem_give(&ble_data_sem);
}

static void process_lte_proto_event(void)
{
	struct cellular_proto_in_event ev;

	int err = k_msgq_get(&lte_proto_msgq, &ev, K_NO_WAIT);
	if (err) {
		LOG_ERR("Error getting lte_proto_event: %d\n", err);
		return;
	}
	LOG_INF("Processed lte_proto_msgq.\n");
	k_sem_give(&lte_proto_sem);

	NofenceMessage proto;
	err = collar_protocol_decode(ev.buf + 2, ev.len - 2, &proto);
	LOG_WRN("Number of received bytes = %d\n", ev.len);

	char *buf = ev.buf;
	for (int i = 0; i < ev.len; i++) {
		printk("\\x%02x", *buf);
		buf++;
	}
	printk("\n");
	if (err) {
		LOG_ERR("Error decoding protobuf message.\n");
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
		uint16_t new_ano_frame = process_ano_msg(&proto.m
								  .ubx_ano_reply);
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
	k_work_queue_start(&poll_q, messaging_poll_thread,
			   K_THREAD_STACK_SIZEOF(messaging_poll_thread),
			   CONFIG_MESSAGING_POLL_THREAD_PRIORITY, NULL);

	k_work_init_delayable(&modem_poll_work, modem_poll_work_fn);
	k_work_schedule_for_queue(&poll_q, &modem_poll_work, K_NO_WAIT);
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



int8_t request_fframe(uint32_t version, uint8_t frame){
	NofenceMessage fence_req;
	proto_InitHeader(&fence_req); /* fill up message header. */
	fence_req.which_m = NofenceMessage_fence_definition_req_tag;
	fence_req.m.fence_definition_req.ulFenceDefVersion = version;
	fence_req.m.fence_definition_req.ucFrameNumber = frame;
	int ret = send_message(&fence_req);
	if (ret){
		LOG_WRN("Failed to send request for frame %d\n", frame);
		return -1;
	}
	return 0;
}

void fence_download( uint8_t new_fframe){
	if (new_fframe == 0 && first_frame){
		first_frame = false;
	}
	else if (new_fframe == 0 && !first_frame){ //something went bad
		expected_fframe = 0;
		new_fence_in_progress = 0;
		return;
	}
	if (new_fframe >= 0){
		if (new_fframe == DOWNLOAD_COMPLETE){
			struct new_fence_available *fence_ready;
			fence_ready = new_new_fence_available();
			fence_ready->fence_version = new_fence_in_progress;
			EVENT_SUBMIT(fence_ready);
			LOG_WRN("Fence %d download "
				"complete!\n", new_fence_in_progress);
			expected_fframe = 0;
			/* trigger ano download for the sake of testing only
			first_ano_frame = true;
			request_ano_frame(0, 0);
			 */
			return;
		}
		if (new_fframe == expected_fframe){
			expected_fframe++;
			/* TODO: handle failure to send request!*/
			request_fframe(new_fence_in_progress, expected_fframe);
			LOG_WRN("Requesting frame %d of new fence: %d"
				".\n", expected_fframe,
				new_fence_in_progress);

		} else{
			expected_fframe = 0;
			new_fence_in_progress = 0;
			return;
		}
	}
}

int8_t request_ano_frame(uint16_t ano_id, uint16_t ano_start){
	NofenceMessage ano_req;
	proto_InitHeader(&ano_req); /* fill up message header. */
	ano_req.which_m = NofenceMessage_ubx_ano_req_tag;
	ano_req.m.ubx_ano_req.usAnoId = ano_id;
	ano_req.m.ubx_ano_req.usStartAno = ano_start;
	int ret = send_message(&ano_req);
	if (ret){
		LOG_WRN("Failed to send request for ano %d\n", ano_start);
		return -1;
	}
	return 0;
}

void ano_download(uint16_t ano_id, uint16_t new_ano_frame){
	LOG_WRN("ano_id = %d, ano_frame = %d\n", ano_id, new_ano_frame);
	if (first_ano_frame){
		new_ano_in_progress = ano_id;
		expected_ano_frame = 0;
		first_ano_frame = false;
	}
	else if (new_ano_frame == 0 && !first_ano_frame){ //something went bad
		expected_ano_frame = 0;
		new_ano_in_progress = 0;
		return;
	}
	if (new_ano_frame >= 0){
		if (new_ano_frame == DOWNLOAD_COMPLETE){
			struct ano_ready *ano_ready;
			ano_ready = new_ano_ready();
			EVENT_SUBMIT(ano_ready);
			LOG_WRN("ANO %d download "
				"complete!\n", new_ano_in_progress);
			return;
		}

		expected_ano_frame += new_ano_frame;
		LOG_WRN("expected ano frame = %d\n",
			expected_ano_frame);
		/* TODO: handle failure to send request!*/
		int ret = request_ano_frame(ano_id,
					 expected_ano_frame);

		LOG_WRN("Requesting frame %d of new ano: %d"
			".\n", expected_ano_frame,
			new_ano_in_progress);

		if (ret != 0){
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
	if (use_server_time){
		msg->header.ulUnixTimestamp = time_from_server;
	}
	else{
		msg->header.ulUnixTimestamp = cached_fix.unixTimestamp;
	}
}

int send_message(NofenceMessage *msg_proto)
{
	static uint8_t waiting_cycles;
	uint8_t encoded_msg[NofenceMessage_size];
	memset(encoded_msg, 0, sizeof(encoded_msg));
	size_t encoded_size = 0;
	LOG_DBG("Start message encoding!, size: %d, version: %u\n",
		sizeof(msg_proto), msg_proto->header.ulVersion);

	LOG_WRN("Input to proto encode: %p,%p,%u,%p\n", msg_proto,
		&encoded_msg[0], sizeof(encoded_msg), &encoded_size);

	int ret = collar_protocol_encode(msg_proto, &encoded_msg[0],
					 sizeof(encoded_msg), &encoded_size);
	for (int i = 0; i < encoded_size; i++) {
		printk("\\x%02x", encoded_msg[i]);
	}
	printk("\n");
	/* add 16-bit uint: size of encoded message. */
	char *tmp;
	tmp = (char *)malloc(encoded_size + 2);
	uint16_t size = BYTESWAP16(encoded_size);
	memcpy(tmp, &size, 2);
	memcpy(tmp + 2, &encoded_msg[0], encoded_size);
	for (int i = 0; i < encoded_size + 2; i++) {
		printk("\\x%02x", tmp[i]);
	}
	printk("\n");
	LOG_DBG("Finished message encoding, %d\n", ret);
	struct messaging_proto_out_event *msg2send =
		new_messaging_proto_out_event();
	LOG_DBG("Declared message event!\n");
	msg2send->buf = tmp;
	LOG_DBG("Message length = %u!\n", encoded_size + 2);
	msg2send->len = encoded_size + 2;
	LOG_DBG("Finished building message event\n!\n");

	EVENT_SUBMIT(msg2send);
	LOG_DBG("Submitted message event\n!\n");

	while (!send_out_ack) {
		k_sleep(K_SECONDS(0.1));
		LOG_WRN("Waiting for ack!\n");
		if (waiting_cycles++ > 10){
			waiting_cycles = 0;
			char *e_msg = "Timed out waiting for cellular ack!\n";
			nf_app_error(ERR_MESSAGING, ETIME,
				     e_msg, strlen(e_msg));
			return -1;
		}
	}
	send_out_ack = false;
	waiting_cycles = 0;
	free(tmp);
	return 0;
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
		/* TODO: publish erase flash event to storage module */
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
		poll_period_minutes = pResp->usPollConnectIntervalSec/60;
		k_work_reschedule_for_queue(&poll_q, &modem_poll_work,
					    K_SECONDS(poll_period_minutes * 60));
		LOG_WRN("Poll period of %d minutes will be used!\n",
			poll_period_minutes);
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
		LOG_WRN("Requesting frame 0 of fence: %d!\n"
			,pResp->ulFenceDefVersion);
		int ret = request_fframe(pResp->ulFenceDefVersion, 0);
		if (ret == 0){
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
	return;
}

uint8_t process_fence_msg(FenceDefinitionResponse *fenceResp)
{
	static uint32_t version;
	static uint8_t total_frames;
	uint8_t frame = fenceResp->ucFrameNumber;
	LOG_WRN("Received frame %d\n", frame);
	if (frame == 0){
		version = fenceResp->ulFenceDefVersion;
		total_frames = fenceResp->ucTotalFrames;
		LOG_WRN("Total number of fence frames = %d\n", total_frames);
		return 0;
	}
	if (new_fence_in_progress != fenceResp->ulFenceDefVersion){
		return 0; //something went wrong, restart fence request
	}
	if (frame == total_frames-1){
		return DOWNLOAD_COMPLETE;
	}
	/* TODO: write "fenceResp" to flash. */
	return frame;
}

uint8_t process_ano_msg(UbxAnoReply *anoResp)
{
	LOG_DBG("Ano response received!\n");
	uint8_t rec_ano_frames = anoResp->rgucBuf.size/sizeof(UBX_MGA_ANO_RAW_t);
	UBX_MGA_ANO_RAW_t *temp = NULL;
	temp = (UBX_MGA_ANO_RAW_t *) (anoResp->rgucBuf.bytes + sizeof(UBX_MGA_ANO_RAW_t));
	uint32_t age = ano_date_to_unixtime_midday(temp->mga_ano.year,
						   temp->mga_ano.month,
						   temp->mga_ano.day);
	LOG_WRN("Relative age of received ANO frame = %d, %d \n",
		age, time_from_server);
	if (age > time_from_server + SECONDS_IN_THREE_DAYS){
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

static uint32_t ano_date_to_unixtime_midday(uint8_t year, uint8_t month, uint8_t day) {
	nf_time_t nf_time;
	nf_time.day = day;
	nf_time.month = month;
	nf_time.year = year + 2000;
	nf_time.hour = 12; //assumed per ANO specs
	nf_time.minute = nf_time.second = 0;
	return time2unix(&nf_time);
}
