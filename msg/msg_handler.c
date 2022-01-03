/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr.h>
#include <zephyr/types.h>
#include <kernel.h>
#include <sys/ring_buffer.h>

#define MODULE msg_handler
#include "module_state_event.h"
#include "peer_conn_event.h"
#include "ble_data_event.h"
#include "msg_data_event.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(MODULE, CONFIG_MSG_LOG_LEVEL);

// static void update_counter_handler(struct k_work *work);
// static struct k_work_delayable update_counter_work;

#define MSG_SIMULATED_THREAD_STACK_SIZE 800
#define MSG_SIMULATED_THREAD_SLEEP 1000
#define MSG_SIMULATED_THREAD_PRIORITY 1

uint16_t counter = 0;

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

/** @brief Function to send dummy data */
static void msg_send(void)
{
	struct msg_data_event *event;
	struct msg_rx_buf *buf;
	int len = sprintf(&buf, "Message counter %d",
			  ++counter); // Send dummy data for test
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

/** @brief Function called in a worker thread to simulate message send event */
// static void update_counter_handler(struct k_work *work)
// {
// 	if (work) {
// 		msg_send();
// 		k_work_reschedule(&update_counter_work, K_SECONDS(1));
// 	}
// }

/** @brief Initialize k_work thread. */
static void init(void)
{
	k_thread_create(&msg_simulated_thread, msg_simulated_thread_stack,
			MSG_SIMULATED_THREAD_STACK_SIZE,
			(k_thread_entry_t)msg_simulated_thread_fn, NULL, NULL,
			NULL, MSG_SIMULATED_THREAD_PRIORITY, 0, K_NO_WAIT);
}

/** @brief Event handler function
  * @param eh Pointer to event handler struct
  * @return true to consume the event (event is not propagated to further listners), false otherwise
  */
static bool event_handler(const struct event_header *eh)
{
	int err;

	if (is_msg_data_event(eh)) {
		const struct msg_data_event *event = cast_msg_data_event(eh);
		LOG_INF("MSG data event sent. Check data in nRF Connect App");
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
			// TODO: Initialize ring buffer here
			// k_work_init_delayable(&update_counter_work,
			// 		      update_counter_handler);
		}

		return false;
	}

	if (is_peer_conn_event(eh)) {
		const struct peer_conn_event *event = cast_peer_conn_event(eh);

		if (event->conn_state == PEER_STATE_CONNECTED) {
			init(); // Start bt message publish thread
			//k_work_reschedule(&update_counter_work, K_SECONDS(1));
		} else {
			//k_work_cancel(&update_counter_work);
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
