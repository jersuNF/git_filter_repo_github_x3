/*
 * Copyright (c) 2021 Nofence AS
 */

#include <kernel.h>
#include <stdio.h>
#include <sys/ring_buffer.h>
#include <zephyr.h>
#include <zephyr/types.h>
#define MODULE msg_handler
#include "ble_data_event.h"
#include "module_state_event.h"
#include "msg_data_event.h"
#include "ble_conn_event.h"

#include "ble_ctrl_event.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(MODULE, CONFIG_MSG_LOG_LEVEL);

#define MSG_SIMULATED_THREAD_STACK_SIZE 800
#define MSG_SIMULATED_THREAD_SLEEP 2000
#define MSG_SIMULATED_THREAD_PRIORITY 1

uint16_t uart_msg_counter = 0;
/* Keep for reference */
/* Define thread stack used for message testing */
// static K_THREAD_STACK_DEFINE(msg_simulated_thread_stack,
// 			     MSG_SIMULATED_THREAD_STACK_SIZE);
// static struct k_thread msg_simulated_thread;

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

/** @brief Function to send dummy data. Keep for reference */
// static void msg_send(void)
// {
// 	struct msg_data_event *event;
// 	struct msg_rx_buf *buf;
// 	int len = sprintf(&buf, "Message counter %d", ++uart_msg_counter);
// 	event = new_msg_data_event();
// 	event->buf = &buf;
// 	event->len = len;
// 	EVENT_SUBMIT(event); // Submit dummy data for event test
// }

// static void msg_simulated_thread_fn(void)
// {
// 	// Start infinite message simulator
// 	while (1) {
// 		//msg_send();
// 		k_sleep(K_MSEC(MSG_SIMULATED_THREAD_SLEEP));
// 	}
// }

/** @brief Initialize ble uart message simulator thread. Keep for reference */
// static void init_msg_uart_simulator(void)
// {
// 	k_thread_create(&msg_simulated_thread, msg_simulated_thread_stack,
// 			MSG_SIMULATED_THREAD_STACK_SIZE,
// 			(k_thread_entry_t)msg_simulated_thread_fn, NULL, NULL,
// 			NULL, MSG_SIMULATED_THREAD_PRIORITY, 0, K_NO_WAIT);
// }

/** @brief Event handler function
 * @param eh Pointer to event handler struct
 * @return true to consume the event (event is not propagated to further
 * listners), false otherwise
 */
static bool event_handler(const struct event_header *eh)
{
	// if (is_msg_data_event(eh)) {
	// 	const struct msg_data_event *event = cast_msg_data_event(eh);
	// 	// LOG_INF("MSG data event sent. Check data in nRF Connect App");
	// 	return true;
	// }

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
			// TODO: DO initialization of module here.
			// I.e initialize ring buffer etc.
			// init_msg_uart_simulator(); // Start bt message publish thread
		}

		return false;
	}

	if (is_ble_conn_event(eh)) {
		const struct ble_conn_event *event = cast_ble_conn_event(eh);

		if (event->conn_state == BLE_STATE_CONNECTED) {
			// Bluetooth connected
		} else {
			// Bluetooth disconnected
		}
		return false;
	}

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, module_state_event);
EVENT_SUBSCRIBE(MODULE, ble_conn_event);
EVENT_SUBSCRIBE(MODULE, ble_data_event);
EVENT_SUBSCRIBE_FINAL(MODULE, msg_data_event);
