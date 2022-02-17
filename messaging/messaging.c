/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include "messaging_module_events.h"
#include "messaging.h"
#include <logging/log.h>
#include "ble_ctrl_event.h"
#include "ble_data_event.h"
#include "lte_proto_event.h"
#include "http_downloader.h"

#include "cellular_controller_events.h"
#include "gps_controller_events.h"
#include "request_events.h"
#include "nf_version.h"
#include "collar_protocol.h"
#define GPS_UBX_NAV_PVT_VALID_HEADVEH_MASK  0x20

uint32_t time_from_server;

K_SEM_DEFINE(cache_lock_sem, 0, 1);

collar_state_struct_t current_state;
gps_last_fix_struct_t cached_fix;
uint8_t poll_period_minutes = 15;
void build_poll_request(NofenceMessage *);
void proto_InitHeader(NofenceMessage *);
void process_poll_response(NofenceMessage *);

_DatePos proto_getLastKnownDatePos(gps_last_fix_struct_t *);
bool proto_hasLastKnownDatePos(const gps_last_fix_struct_t *);

bool m_confirm_acc_limits, m_confirm_ble_key, m_transfer_boot_params;
bool send_out_ack;

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

static struct k_work_delayable modem_poll_work;
void modem_poll_work_fn()
{
	/* Add logic for the periodic protobuf modem poller. */
	NofenceMessage new_msg;
	if (!k_sem_take(&cache_lock_sem, K_SECONDS(1))) {
		build_poll_request(&new_msg);
		k_sem_give(&cache_lock_sem);
	}
	send_message(&new_msg);

//	k_work_reschedule(&modem_poll_work,
//			  K_SECONDS(CONFIG_PROTO_POLLER_WORK_SEC));
	k_work_reschedule(&modem_poll_work,
			  K_SECONDS(poll_period_minutes*60));
}

void messaging_module_init(void)
{
	k_work_init_delayable(&modem_poll_work, modem_poll_work_fn);
	modem_poll_work_fn();
//	k_work_reschedule(&modem_poll_work,
//			  K_SECONDS(poll_period_minutes*60));
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
    size_t len;
    uint8_t *pMsg = NULL;
    uint8_t *buf = NULL;

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
	if (is_lte_proto_event(eh)) {
		struct lte_proto_event *ev = cast_lte_proto_event(eh);
		while (k_msgq_put(&lte_proto_msgq, buf, K_NO_WAIT) != 0) {
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
		return false;
	}
	if (is_update_flash_erase(eh)) {
		current_state.flash_erase_count ++;
		return false;
	}
	if (is_update_zap_count(eh)) {
		struct update_zap_count *ev = cast_update_zap_count(eh);
		current_state.zap_count = ev->count;
		return false;
	}
	if (is_cellular_ack_event(eh)){
		send_out_ack = true;
		return false;
	}
	if (is_new_gps_fix(eh)){
		struct new_gps_fix *ev = cast_new_gps_fix(eh);
		if (!k_sem_take(&cache_lock_sem, K_SECONDS(0.1))) {
			cached_fix = ev->fix;
			k_sem_give(&cache_lock_sem);
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
EVENT_SUBSCRIBE(MODULE,cellular_ack_event);
EVENT_SUBSCRIBE(MODULE, update_collar_mode);
EVENT_SUBSCRIBE(MODULE, update_collar_status);
EVENT_SUBSCRIBE(MODULE, update_fence_status);
EVENT_SUBSCRIBE(MODULE, update_fence_version);
EVENT_SUBSCRIBE(MODULE, update_flash_erase);
EVENT_SUBSCRIBE(MODULE, update_zap_count);
EVENT_SUBSCRIBE(MODULE, new_gps_fix);

static inline void process_ble_ctrl_event(void)
{
	struct ble_ctrl_event ev;

	int err = k_msgq_get(&ble_ctrl_msgq, &ev, K_NO_WAIT);
	if (err) {
		LOG_ERR("Error getting ble_ctrl_event: %d", err);
		return;
	}
	LOG_INF("Processed ble_ctrl_event.");
	k_sem_give(&ble_ctrl_sem);
}

static inline void process_ble_data_event(void)
{
	struct ble_data_event ev;

	int err = k_msgq_get(&ble_data_msgq, &ev, K_NO_WAIT);
	if (err) {
		LOG_ERR("Error getting ble_data_event: %d", err);
		return;
	}
	LOG_INF("Processed ble_data_event.");
	k_sem_give(&ble_data_sem);
}

static inline void process_lte_proto_event(void)
{
	struct lte_proto_event ev;

	int err = k_msgq_get(&lte_proto_msgq, &ev, K_NO_WAIT);
	if (err) {
		LOG_ERR("Error getting lte_proto_event: %d", err);
		return;
	}
	LOG_INF("Processed lte_proto_msgq.");
	k_sem_give(&lte_proto_sem);

	NofenceMessage proto;
	err = collar_protocol_decode(ev.buf, ev.len, &proto);
	if (err) {
		LOG_ERR("Error decoding protobuf message.");
		return;
	}
	struct messaging_ack_event *ack = new_messaging_ack_event();
	EVENT_SUBMIT(ack);
	/* process poll response */
	if (proto.which_m == NofenceMessage_poll_message_resp_tag) {
		process_poll_response(&proto);
		return;
	}
	else if (proto.which_m == NofenceMessage_fence_definition_resp_tag) {
		process_fence_msg();
		return;
	}
	else if (proto.which_m == NofenceMessage_ubx_ano_reply_tag) {
		process_ano_msg();
		return;
	}
	else{
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

K_THREAD_DEFINE(messaging_thread, CONFIG_MESSAGING_THREAD_SIZE,
		messaging_thread_fn, NULL, NULL, NULL,
		CONFIG_MESSAGING_THREAD_PRIORITY, 0, 0);

void build_poll_request(NofenceMessage *poll_req)
{
	proto_InitHeader(poll_req); /* fill up message header. */
	poll_req->which_m = NofenceMessage_poll_message_req_tag;
	poll_req->m.poll_message_req.datePos = proto_getLastKnownDatePos(&cached_fix);
	poll_req->m.poll_message_req.has_datePos =
		proto_hasLastKnownDatePos(&cached_fix);
	poll_req->m.poll_message_req.eMode = current_state.collar_mode;
	poll_req->m.poll_message_req.usZapCount = current_state.zap_count;
	poll_req->m.poll_message_req.eCollarStatus = current_state.collar_status;
	poll_req->m.poll_message_req.eFenceStatus = current_state.fence_status;
	poll_req->m.poll_message_req.ulFenceDefVersion = current_state
								.fence_version;
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
uint32_t dummy_timestamp = 1644913291;
void proto_InitHeader(NofenceMessage * msg) {
	msg->header.ulId = 11500; //TODO: read from eeprom
	msg->header.ulVersion = NF_X25_VERSION_NUMBER;
	msg->header.has_ulVersion = true;
	msg->header.ulUnixTimestamp = dummy_timestamp;
}

int send_message(NofenceMessage *msg_proto){
	uint8_t encoded_msg[NofenceMessage_size];
	size_t *encoded_size = NULL;

	int ret = collar_protocol_encode(msg_proto, encoded_msg,
					 sizeof(encoded_msg), encoded_size);
	struct messaging_proto_out_event *msg2send =
		new_messaging_proto_out_event();
	msg2send->buf = &encoded_msg[0];
	msg2send->len = *encoded_size;
	EVENT_SUBMIT(msg2send);
	while(!send_out_ack){
		k_sleep(K_SECONDS(0.1));
	}
	send_out_ack = false;
	return 0;
}

void process_poll_response(NofenceMessage *proto){
	PollMessageResponse *pResp = &proto-> m.poll_message_resp;
	if (pResp->has_xServerIp && strlen(pResp->xServerIp) > 0) {
		struct messaging_host_address_event *host_add_event =
			new_messaging_host_address_event();
//		strcpy(host_add_event.address, pResp->xServerIp, sizeof(pResp->xServerIp));
		strncpy(host_add_event->address, pResp->xServerIp, sizeof
			(pResp->xServerIp));
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
		time_from_server = proto->header.ulUnixTimestamp;
	}
	if (pResp->has_usPollConnectIntervalSec) {
		poll_period_minutes = pResp->usPollConnectIntervalSec;
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
		process_upgrade_request(pResp->versionInfo);
	}
	if (pResp->ulFenceDefVersion) {
		/* TODO: Submit pResp->ulFenceDefVersion to AMC */
	}
	return;
}

/* @brief: starts a firmware download if a new version exists on the server. */
void process_upgrade_request(VersionInfoFW *fw_ver_from_server){
	// compare versions and start update when needed.//
	return;
}

void process_fence_msg(void){
	return;
}

void process_ano_msg(void){
	return;
}

_DatePos proto_getLastKnownDatePos(gps_last_fix_struct_t *gpsLastFix) {

	bool valid_headVeh = (bool) (gpsLastFix->pvt_flags & GPS_UBX_NAV_PVT_VALID_HEADVEH_MASK);
	_DatePos a =  {
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
#if defined (HW_BAROMETER)
		.has_usHeight = true,
				.usHeight = gpsLastFix->baro_Height,
#endif
		.has_ucGpsMode = true,
		.ucGpsMode = gpsLastFix->gps_mode
	};
	return a;
}

bool proto_hasLastKnownDatePos(const gps_last_fix_struct_t * gps) {
	return gps->unixTimestamp != 0;

}