/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/types.h>
#include <sys/ring_buffer.h>

#define MODULE msg_handler
#include "module_state_event.h"
#include "peer_conn_event.h"
#include "ble_data_event.h"
#include "msg_data_event.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(MODULE, CONFIG_MSG_LOG_LEVEL);

#define MSG_SIMULATED_THREAD_STACK_SIZE 800
#define MSG_SIMULATED_THREAD_SLEEP 1000
#define MSG_SIMULATED_THREAD_PRIORITY 1

static K_THREAD_STACK_DEFINE(msg_simulated_thread_stack,
			     MSG_SIMULATED_THREAD_STACK_SIZE);
static struct k_thread msg_simulated_thread;

#define MSG_BUF_SIZE CONFIG_MSG_BUF_SIZE

struct msg_rx_buf {
	atomic_t ref_counter;
	size_t len;
	uint8_t buf[MSG_BUF_SIZE];
};

struct msg_tx_buf {
	struct ring_buf rb;
	uint32_t buf[MSG_BUF_SIZE];
};

uint16_t counter = 0;
static void msg_send(void)
{
	struct msg_data_event *event;
	struct msg_rx_buf *buf;
	int len = sprintf(&buf, "Message counter %d", ++counter);
	event = new_msg_data_event();
	event->buf = &buf;
	event->len = len;
	EVENT_SUBMIT(event);
}

static void msg_simulated_thread_fn(void)
{
	while (true) {
		msg_send();
		k_sleep(K_MSEC(MSG_SIMULATED_THREAD_SLEEP));
	}
}

static void init(void)
{
	k_thread_create(&msg_simulated_thread, msg_simulated_thread_stack,
			MSG_SIMULATED_THREAD_STACK_SIZE,
			(k_thread_entry_t)msg_simulated_thread_fn, NULL, NULL,
			NULL, MSG_SIMULATED_THREAD_PRIORITY, 0, K_NO_WAIT);
}

static bool event_handler(const struct event_header *eh)
{
	int err;

	if (is_msg_data_event(eh)) {
		const struct msg_data_event *event = cast_msg_data_event(eh);

		LOG_INF("MSG data event sent. Check data in nRF Connect");

		return true;
	}

	if (is_ble_data_event(eh)) {
		const struct ble_data_event *event = cast_ble_data_event(eh);

		LOG_INF("BLE DATA event received");
		LOG_HEXDUMP_INF(event->buf, event->len, "DATA: ");
		return false;
	}

	if (is_module_state_event(eh)) {
		const struct module_state_event *event =
			cast_module_state_event(eh);

		if (check_state(event, MODULE_ID(main), MODULE_STATE_READY)) {
			// Initialize ring buffer here
			init();
		}

		return false;
	}

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}
EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, module_state_event);
EVENT_SUBSCRIBE(MODULE, peer_conn_event);
EVENT_SUBSCRIBE(MODULE, ble_data_event);
EVENT_SUBSCRIBE_FINAL(MODULE, msg_data_event);
