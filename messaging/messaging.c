/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include "messaging.h"
#include <logging/log.h>
#include "ble_ctrl_event.h"
#include "ble_data_event.h"
#include "lte_proto_event.h"
#include "collar_protocol.h"
#include "fw_upgrade_events.h"

#include "nf_version.h"

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
	k_work_reschedule(&modem_poll_work,
			  K_SECONDS(CONFIG_PROTO_POLLER_WORK_SEC));
}

void messaging_module_init(void)
{
	k_work_init_delayable(&modem_poll_work, modem_poll_work_fn);
	k_work_reschedule(&modem_poll_work,
			  K_SECONDS(CONFIG_PROTO_POLLER_WORK_SEC));
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
	//int err;
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
		while (k_msgq_put(&lte_proto_msgq, ev, K_NO_WAIT) != 0) {
			k_msgq_purge(&lte_proto_msgq);
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

	/* Decode protobuf message and submit FOTA event if a new version
	 * is available. 
	 */
	NofenceMessage proto;
	err = collar_protocol_decode(ev.buf, ev.len, &proto);
	if (err) {
		LOG_ERR("Error decoding protobuf message.");
		return;
	}

	if (proto.m.firmware_upgrade_req.ulVersion > NF_X25_VERSION_NUMBER) {
		struct start_fota_event *ev = new_start_fota_event();
		ev->override_default_host = false;
		ev->version = proto.m.firmware_upgrade_req.ulVersion;
		EVENT_SUBMIT(ev);
	} else {
		LOG_WRN("Requested firmware version is \
			 same or older than current");
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