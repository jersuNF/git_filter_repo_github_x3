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
RING_BUF_DECLARE(ble_receive_ring_buf,256);

static uint8_t diagnostics_up_data_buffer[CONFIG_DIAGNOSTICS_RTT_BUFFER_SIZE];
static uint8_t diagnostics_down_data_buffer[CONFIG_DIAGNOSTICS_RTT_BUFFER_SIZE];

static void diagnostics_rtt_set_active(void);
static bool diagnostics_rtt_is_active(void);
static void diagnostics_handler(void);

static void diagnostics_parse_input(uint8_t* data, uint32_t size);
static void diagnostics_send(uint8_t* data, uint32_t size);
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

			diagnostics_parse_input(buffer, bytes_read);
		}
		if (!ring_buf_is_empty(&ble_receive_ring_buf)) {
			uint8_t* ble_data;
			uint32_t size = ring_buf_get_claim(&ble_receive_ring_buf, &ble_data, 100);

			//diagnostics_parse_input(ble_data, size);

			memcpy(buffer, ble_data, size);
			buffer[size] = '\0';
			diagnostics_log(DIAGNOSTICS_WARNING, "BLE_DATA", buffer);

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

static void diagnostics_parse_input(uint8_t* data, uint32_t size)
{
	for (uint32_t i = 0; i < size; i++) {
		if ((data[i] == '\n') || (data[i] == '\r') || (data[i] == 'U')) {
			diagnostics_send("Zephyr version is ", strlen("Zephyr version is "));
			diagnostics_send(KERNEL_VERSION_STRING, strlen(KERNEL_VERSION_STRING));
			diagnostics_send("\r\n", strlen("\r\n"));
		}
	}
}

static void diagnostics_send(uint8_t* data, uint32_t size)
{
	if (k_mutex_lock(&rtt_send_mutex, K_MSEC(10)) == 0) {
		if (diagnostics_rtt_is_active()) {
			diagnostics_rtt_set_active();
			uint32_t was_copied = SEGGER_RTT_WriteSkipNoLock(CONFIG_DIAGNOSTICS_RTT_UP_CHANNEL_INDEX, (const char*)data, size);
			if (!was_copied) {
				LOG_ERR("Failed to send diagnostics message.");
			}
		}

		k_mutex_unlock(&rtt_send_mutex);
	} else {
		LOG_ERR("Failed to send diagnostics message to RTT.");
	}

	if (atomic_test_bit(&ble_is_connected, 0)) {

	}
}

static void diagnostics_log(enum diagnostics_severity severity, 
						    char* header, char* msg)
{
	uint32_t used_size = 0;
	int64_t uptime = k_uptime_get();
	char uptime_str[13+1];
	used_size = snprintf(uptime_str, 13+1, "%llx", uptime);
	if ((used_size < 0) || (used_size >= sizeof(uptime_str))) {
		LOG_ERR("Uptime encoding error.");
		return;
	}

	/* Color code + bracket pair + max timestamp + color code + header_size + ": " + color code + msg_size + linefeed + color code */
	uint32_t bufsiz = 7 + 2 + 13 + 1 + 7 + strlen(header) + 2 + 7 + strlen(msg) + 2 + 7;
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
	used_size = snprintf(buf, bufsiz, "%s[%13s] %s%s: %s%s%s\r\n", 
							RTT_CTRL_TEXT_BRIGHT_GREEN, uptime_str, 
							severity_color, header, 
							RTT_CTRL_RESET, msg,
							RTT_CTRL_RESET);
	if ((used_size < 0) || (used_size >= bufsiz)) {
		LOG_ERR("Message encoding error.");

		k_free(buf);

		return;
	}
	
	diagnostics_send(buf, strlen(buf));

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
			LOG_ERR("BLE CONNECTED!");
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

void event_manager_trace_event_execution(const struct event_header *eh,
				  bool is_start)
{
	char buf[100];

	eh->type_id->log_event(eh, buf, sizeof(buf));
	diagnostics_log(DIAGNOSTICS_INFO, is_start ? "EVENT_MANAGER_EXEC_START" : "EVENT_MANAGER_EXEC_STOP", buf);
}

void event_manager_trace_event_submission(const struct event_header *eh,
				const void *trace_info)
{
	char buf[100];

	eh->type_id->log_event(eh, buf, sizeof(buf));
	diagnostics_log(DIAGNOSTICS_INFO, "EVENT_MANAGER_SUBMIT", buf);
}

#endif