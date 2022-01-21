/*
 * Copyright (c) 2022 Nofence AS
 */

#include <zephyr.h>
#include "diagnostics_events.h"
#include "diagnostics.h"
#include <logging/log.h>
#include <sys/ring_buffer.h>
#include <stdio.h>

#include "nf_version.h"
#include "version.h"

#include "msg_data_event.h"

#include "ble_ctrl_event.h"
#include "ble_data_event.h"
#include "ble_conn_event.h"

#define LOG_MODULE_NAME diagnostics
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_DIAGNOSTICS_LOG_LEVEL);

K_MUTEX_DEFINE(rtt_send_mutex);
K_TIMER_DEFINE(rtt_activity_timer, NULL, NULL);

#define THREAD_PRIORITY 7

extern const k_tid_t diag_handler_id; 

atomic_t ble_is_connected = ATOMIC_INIT(0);
RING_BUF_DECLARE(ble_receive_ring_buf, CONFIG_DIAGNOSTICS_PARSE_BUFFER_LENGTH);

static uint8_t diagnostics_up_data_buffer[CONFIG_DIAGNOSTICS_RTT_BUFFER_SIZE];
static uint8_t diagnostics_down_data_buffer[CONFIG_DIAGNOSTICS_RTT_BUFFER_SIZE];

static uint8_t rtt_parsing_buffer[CONFIG_DIAGNOSTICS_PARSE_BUFFER_LENGTH];
static uint32_t rtt_parsing_count = 0;

static void diagnostics_rtt_set_active(void);
static bool diagnostics_rtt_is_active(void);
static void diagnostics_handler(void);

static uint32_t diagnostics_parse_input(enum diagnostics_interface interface, 
					uint8_t* data, uint32_t size);
					
static void diagnostics_send_rtt(uint8_t* data, uint32_t size);
static void diagnostics_send_ble(uint8_t* data, uint32_t size);
static void diagnostics_send(enum diagnostics_interface interface, 
			     uint8_t* data, uint32_t size);

static void diagnostics_log(enum diagnostics_severity severity, 
						    char* header, char* msg);

int diagnostics_module_init()
{
	SEGGER_RTT_Init();

	if (SEGGER_RTT_ConfigUpBuffer(CONFIG_DIAGNOSTICS_RTT_UP_CHANNEL_INDEX, "DIAG_UP", diagnostics_up_data_buffer, CONFIG_DIAGNOSTICS_RTT_BUFFER_SIZE, SEGGER_RTT_MODE_NO_BLOCK_SKIP) != 0) {
		LOG_ERR("Diagnostics module failed to setup RTT up channel.");
	}
	if (SEGGER_RTT_ConfigDownBuffer(CONFIG_DIAGNOSTICS_RTT_DOWN_CHANNEL_INDEX, "DIAG_DOWN", diagnostics_down_data_buffer, CONFIG_DIAGNOSTICS_RTT_BUFFER_SIZE, SEGGER_RTT_MODE_NO_BLOCK_SKIP) != 0) {
		LOG_ERR("Diagnostics module failed to setup RTT down channel.");
	}

	diagnostics_rtt_set_active();
	k_thread_start(diag_handler_id);

	return 0;
}

static void diagnostics_rtt_set_active(void)
{
	k_timer_start(&rtt_activity_timer, K_MSEC(CONFIG_DIAGNOSTICS_RTT_TIMEOUT_MS), K_NO_WAIT);
}

static bool diagnostics_rtt_is_active(void)
{
	return (k_timer_remaining_ticks(&rtt_activity_timer) != 0);
}

static void diagnostics_handler(void)
{
	uint8_t buffer[100];

	while (true) {
		bool processed_data = false;
		if (SEGGER_RTT_HASDATA(CONFIG_DIAGNOSTICS_RTT_DOWN_CHANNEL_INDEX)) {
			diagnostics_rtt_set_active();

			size_t bytes_read = SEGGER_RTT_Read(CONFIG_DIAGNOSTICS_RTT_DOWN_CHANNEL_INDEX, buffer, 100);

			diagnostics_parse_input(DIAGNOSTICS_RTT, buffer, bytes_read);
		}
		if (!ring_buf_is_empty(&ble_receive_ring_buf)) {
			uint8_t* ble_data;
			uint32_t size = ring_buf_get_claim(
						&ble_receive_ring_buf, 
						&ble_data, 100);

			diagnostics_parse_input(DIAGNOSTICS_BLE, 
						ble_data, size);

			if (ring_buf_get_finish(&ble_receive_ring_buf, size) != 0) {
				LOG_ERR("Failed to free ring buffer memory for BLE.");
			}
		}

		if (!processed_data) {
			if (diagnostics_rtt_is_active()) {
				k_msleep(10);
			} else {
				k_msleep(1000);
			}
		}
	}
}

static uint32_t diagnostics_parse_input(enum diagnostics_interface interface, 
					uint8_t* data, uint32_t size)
{
	uint32_t bytes_parsed = 0;
	for (uint32_t i = 0; i < size; i++) {
		if ((data[i] == '\n') || (data[i] == '\r') || (data[i] == 'U')) {
			diagnostics_send(interface, "Zephyr version is ", strlen("Zephyr version is "));
			diagnostics_send(interface, KERNEL_VERSION_STRING, strlen(KERNEL_VERSION_STRING));
			diagnostics_send(interface, "\r\n", strlen("\r\n"));
		}
	}
	bytes_parsed = size;

	return size;
}

static void diagnostics_send_rtt(uint8_t* data, uint32_t size)
{
	if (k_mutex_lock(&rtt_send_mutex, K_MSEC(10)) != 0) {
		LOG_ERR("Failed to send diagnostics message to RTT.");
		return;
	}
	
	diagnostics_rtt_set_active();
	uint32_t was_copied = SEGGER_RTT_WriteSkipNoLock(
				CONFIG_DIAGNOSTICS_RTT_UP_CHANNEL_INDEX, 
				(const char*)data, size);
	if (!was_copied) {
		LOG_ERR("Failed to send diagnostics message.");
	}

	k_mutex_unlock(&rtt_send_mutex);
}

static void diagnostics_send_ble(uint8_t* data, uint32_t size)
{
	struct msg_data_event *msg = new_msg_data_event(size);
	memcpy(msg->dyndata.data, data, size);
	EVENT_SUBMIT(msg);
}

static void diagnostics_send(enum diagnostics_interface interface, 
			     uint8_t* data, uint32_t size)
{
	if ((interface == DIAGNOSTICS_RTT) || (interface == DIAGNOSTICS_ALL)) {
		if (diagnostics_rtt_is_active()) {
			diagnostics_send_rtt(data, size);
		}
	}

	if ((interface == DIAGNOSTICS_BLE) || (interface == DIAGNOSTICS_ALL)) {
		if (atomic_test_bit(&ble_is_connected, 0)) {
			diagnostics_send_ble(data, size);
		}
	}
}

static void diagnostics_log(enum diagnostics_severity severity, 
						    char* header, char* msg)
{
	uint32_t used_size = 0;
	uint32_t uptime = k_uptime_get_32();
	if (uptime > 100*365*24*3600*1000) {
		LOG_ERR("Uptime is more than 100 years.");
		uptime = uptime % 100*365*24*3600*1000;
	}
	char uptime_str[13+2];
	used_size = snprintf(uptime_str, 13+2, "%u.%03u", uptime/1000, uptime%1000);
	if ((used_size < 0) || (used_size >= sizeof(uptime_str))) {
		LOG_ERR("Uptime encoding error.");
		return;
	}

	/* Color code + bracket pair + max timestamp + color code + header_size + ": " + color code + msg_size + linefeed + color code */
	uint32_t bufsiz = 7 + 2 + 13 + 2 + 7 + strlen(header) + 2 + 7 + strlen(msg) + 2 + 7;
	char* buf = k_malloc(bufsiz);
	if (buf == NULL) {
		LOG_ERR("Failed to allocated memory.");
		return;
	}

	const char* severity_color = RTT_CTRL_TEXT_BRIGHT_WHITE;
	if (severity == DIAGNOSTICS_WARNING) {
		severity_color = RTT_CTRL_TEXT_BRIGHT_YELLOW;
	} else if (severity == DIAGNOSTICS_ERROR) {
		severity_color = RTT_CTRL_TEXT_BRIGHT_RED;
	}
	used_size = snprintf(buf, bufsiz, "%s[%14s] %s%s: %s%s%s\r\n", 
							RTT_CTRL_TEXT_BRIGHT_GREEN, uptime_str, 
							severity_color, header, 
							RTT_CTRL_RESET, msg,
							RTT_CTRL_RESET);
	if ((used_size < 0) || (used_size >= bufsiz)) {
		LOG_ERR("Message encoding error.");

		k_free(buf);

		return;
	}
	
	diagnostics_send(DIAGNOSTICS_RTT, buf, strlen(buf));

	k_free(buf);
}

/**
 * @brief Main event handler function. 
 *        TODO: Add more...
 * 
 * @param[in] eh Event_header for the if-chain to 
 *               use to recognize which event triggered.
 * 
 * @return True or false based on if we want to consume the event or not.
 */
static bool event_handler(const struct event_header *eh)
{
	if (is_ble_conn_event(eh)) {
		const struct ble_conn_event *event = cast_ble_conn_event(eh);
		if (event->conn_state == BLE_STATE_CONNECTED) {
			atomic_set(&ble_is_connected, 1);
		} else {
			atomic_set(&ble_is_connected, 0);
		}
		return false;
	}

	if (is_ble_data_event(eh)) {
		const struct ble_data_event *event = cast_ble_data_event(eh);

		uint32_t sent_size = ring_buf_put(&ble_receive_ring_buf, event->buf, event->len);
		if (sent_size != event->len) {
			LOG_ERR("BLE receive data overflow.");
		}

		// TODO - Trigger task in case of sleep mechanisms? 

		return false;
	}

	return false;
}

K_THREAD_DEFINE(diag_handler_id, CONFIG_DIAGNOSTICS_STACK_SIZE, diagnostics_handler, NULL, NULL, NULL,
		THREAD_PRIORITY, 0, K_TICKS_FOREVER);

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, ble_conn_event);
EVENT_SUBSCRIBE(MODULE, ble_data_event);


#if CONFIG_DIAGNOSTICS_PROFILE_EVENTS

int event_manager_trace_event_init(void)
{
	diagnostics_log(DIAGNOSTICS_INFO, "EVENT_MANAGER_INIT", "Initializing");
	return 0;
}

static void diagnostics_build_event_string(const struct event_header *eh, uint8_t* buffer, uint32_t size)
{
	/* Must have space for event name, 
	parantheses, space and null-termination. */
	if (size < (strlen(eh->type_id->name) + 4)) {
		LOG_ERR("Too small buffer for diagnostics logging of event.");
	}

	uint32_t length = snprintf(buffer, size, "(%s) ", eh->type_id->name);
	eh->type_id->log_event(eh, &buffer[length], size-length);
}

void event_manager_trace_event_execution(const struct event_header *eh,
				  bool is_start)
{
	char buf[100];
	diagnostics_build_event_string(eh, buf, sizeof(buf));
	diagnostics_log(DIAGNOSTICS_INFO, is_start ? "EVENT_MANAGER_EXEC_START" : "EVENT_MANAGER_EXEC_STOP", buf);
}

void event_manager_trace_event_submission(const struct event_header *eh,
				const void *trace_info)
{
	char buf[100];
	diagnostics_build_event_string(eh, buf, sizeof(buf));
	diagnostics_log(DIAGNOSTICS_INFO, "EVENT_MANAGER_SUBMIT", buf);
}

#endif