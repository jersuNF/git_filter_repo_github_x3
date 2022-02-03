/*
 * Copyright (c) 2022 Nofence AS
 */

#include "diagnostics.h"
#include "diagnostics_events.h"
#include "diagnostics_types.h"
#include "parser.h"
#include "passthrough.h"

#include <zephyr.h>
#include <logging/log.h>
#include <stdio.h>
#include <stdlib.h>

#include "msg_data_event.h"

#include "ble_ctrl_event.h"
#include "ble_data_event.h"
#include "ble_conn_event.h"

#define LOG_MODULE_NAME diagnostics
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_DIAGNOSTICS_LOG_LEVEL);

#define THREAD_PRIORITY 7

extern const k_tid_t diag_handler_id; 

/* BLE variables */
atomic_t ble_is_connected = ATOMIC_INIT(0);

K_MUTEX_DEFINE(ble_receive_buffer_mutex);
uint8_t ble_receive_buffer[CONFIG_DIAGNOSTICS_RECEIVE_BUFFER_LENGTH];
uint32_t ble_received_count = 0;
atomic_t ble_has_new_data = ATOMIC_INIT(0);

/* RTT variables */
static uint8_t rtt_up_data_buffer[CONFIG_DIAGNOSTICS_RTT_BUFFER_SIZE];
static uint8_t rtt_down_data_buffer[CONFIG_DIAGNOSTICS_RTT_BUFFER_SIZE];

uint8_t rtt_receive_buffer[CONFIG_DIAGNOSTICS_RECEIVE_BUFFER_LENGTH];
uint32_t rtt_received_count = 0;

K_MUTEX_DEFINE(rtt_send_mutex);
K_TIMER_DEFINE(rtt_activity_timer, NULL, NULL);

/* Private function declarations */
static void diagnostics_handler(void);

static void diagnostics_handle_rtt_input(bool* got_data);
static void diagnostics_handle_ble_input(bool* got_data);
static void diagnostics_handle_passthrough_input(bool* got_data);

static uint32_t diagnostics_buffer_consume(uint8_t* buffer, 
					   uint32_t consumed, 
					   uint32_t total_bytes);

static void diagnostics_rtt_set_active(void);
static bool diagnostics_rtt_is_active(void);

static uint32_t diagnostics_process_input(enum diagnostics_interface interface,
					  uint8_t* data, uint32_t size);
					
static void diagnostics_send_rtt(const uint8_t* data, uint32_t size);
static void diagnostics_send_ble(const uint8_t* data, uint32_t size);
static void diagnostics_send(enum diagnostics_interface interface, 
			     const uint8_t* data, uint32_t size);

static void diagnostics_log(enum diagnostics_severity severity, 
			    char* header, char* msg);

/* Public function implementations */
int diagnostics_module_init()
{
	SEGGER_RTT_Init();

	if (SEGGER_RTT_ConfigUpBuffer(CONFIG_DIAGNOSTICS_RTT_UP_CHANNEL_INDEX, 
				      "DIAG_UP", 
				      rtt_up_data_buffer, 
				      CONFIG_DIAGNOSTICS_RTT_BUFFER_SIZE, 
				      SEGGER_RTT_MODE_NO_BLOCK_SKIP) != 0) {
		LOG_ERR("Diagnostics module failed to setup RTT up channel.");
	}
	if (SEGGER_RTT_ConfigDownBuffer(CONFIG_DIAGNOSTICS_RTT_DOWN_CHANNEL_INDEX, 
					"DIAG_DOWN", 
					rtt_down_data_buffer, 
					CONFIG_DIAGNOSTICS_RTT_BUFFER_SIZE, 
					SEGGER_RTT_MODE_NO_BLOCK_SKIP) != 0) {
		LOG_ERR("Diagnostics module failed to setup RTT down channel.");
	}

	diagnostics_rtt_set_active();
	k_thread_start(diag_handler_id);


	struct parser_action actions = {
		.send_resp = diagnostics_send,
		.thru_enable = passthrough_enable
	};
	parser_init(&actions);

	return 0;
}

/* Private function implementations */
static void diagnostics_handler(void)
{
	while (true) {
		bool rtt_got_data = false;
		diagnostics_handle_rtt_input(&rtt_got_data);

		bool ble_got_data = false;
		diagnostics_handle_ble_input(&ble_got_data);

		bool passthrough_got_data = false;
		diagnostics_handle_passthrough_input(&passthrough_got_data);

		/* Only delay when no data was processed*/
		if (!(rtt_got_data || ble_got_data || passthrough_got_data)) {
			/* Sleep longer when RTT is inactive */
			if (diagnostics_rtt_is_active()) {
				k_msleep(10);
			} else {
				k_msleep(1000);
			}
		}
	}
}

static void diagnostics_handle_rtt_input(bool* got_data)
{
	if (!SEGGER_RTT_HASDATA(CONFIG_DIAGNOSTICS_RTT_DOWN_CHANNEL_INDEX)) {
		/* No new data */
		return;
	}
	diagnostics_rtt_set_active();

	size_t bytes_read = SEGGER_RTT_Read(
				CONFIG_DIAGNOSTICS_RTT_DOWN_CHANNEL_INDEX, 
				&rtt_receive_buffer[rtt_received_count], 
				CONFIG_DIAGNOSTICS_RECEIVE_BUFFER_LENGTH - rtt_received_count);

	if (bytes_read > 0) {
		*got_data = true;

		rtt_received_count += bytes_read;
		uint32_t parsed_bytes = diagnostics_process_input(
							DIAGNOSTICS_RTT, 
							rtt_receive_buffer, 
							rtt_received_count);
		
		if (parsed_bytes > 0)
		{
			diagnostics_buffer_consume(rtt_receive_buffer, 
						   parsed_bytes, 
						   rtt_received_count);
		}
	}
}

static void diagnostics_handle_ble_input(bool* got_data)
{
	if (!atomic_test_bit(&ble_has_new_data, 0)) {
		/* No new data */
		return;
	}
	if (k_mutex_lock(&ble_receive_buffer_mutex, K_MSEC(10)) == 0) {
		uint32_t parsed_bytes = 
			diagnostics_process_input(DIAGNOSTICS_BLE, 
				ble_receive_buffer, ble_received_count);

		if (parsed_bytes > 0)
		{
			*got_data = true;

			ble_received_count = diagnostics_buffer_consume(
							ble_receive_buffer, 
							parsed_bytes, 
							ble_received_count);
		}

		atomic_clear_bit(&ble_has_new_data, 0);

		k_mutex_unlock(&ble_receive_buffer_mutex);
	}
}

static void diagnostics_handle_passthrough_input(bool* got_data)
{
	uint8_t* data;
	uint32_t size = 0;

	enum diagnostics_interface passthrough_interface = DIAGNOSTICS_NONE;
	passthrough_get_enabled_interface(&passthrough_interface);
	if (passthrough_interface == DIAGNOSTICS_NONE) {
		/* Passthrough not enabled */
		return;
	}

	size = passthrough_claim_read_data(&data);

	if (size > 0) {
		*got_data = true;

		diagnostics_send(passthrough_interface, data, size);
		passthrough_finish_read_data(size);
	}
}

static uint32_t diagnostics_process_input(enum diagnostics_interface interface, 
					uint8_t* data, uint32_t size)
{
	uint32_t bytes_parsed = 0;
	enum diagnostics_interface passthrough_interface = DIAGNOSTICS_NONE;
	passthrough_get_enabled_interface(&passthrough_interface);

	if (passthrough_interface != DIAGNOSTICS_NONE) {
		bytes_parsed = passthrough_write_data(data, size);
	} else {
		parser_handle(interface, data, size);
	}
	return bytes_parsed;
}

static uint32_t diagnostics_buffer_consume(uint8_t* buffer, uint32_t consumed, uint32_t total_bytes)
{
	/* Move unparsed data to start of buffer */
	for (uint32_t i = 0; i < (total_bytes-consumed); i++)
	{
		buffer[i] = buffer[consumed+i];
	}

	return total_bytes - consumed;
}

static void diagnostics_rtt_set_active(void)
{
	k_timer_start(&rtt_activity_timer, K_MSEC(CONFIG_DIAGNOSTICS_RTT_TIMEOUT_MS), K_NO_WAIT);
}

static bool diagnostics_rtt_is_active(void)
{
	return (k_timer_remaining_ticks(&rtt_activity_timer) != 0);
}

static void diagnostics_send_rtt(const uint8_t* data, uint32_t size)
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

static void diagnostics_send_ble(const uint8_t* data, uint32_t size)
{
	struct msg_data_event *msg = new_msg_data_event(size);
	memcpy(msg->dyndata.data, data, size);
	EVENT_SUBMIT(msg);
}

static void diagnostics_send(enum diagnostics_interface interface, 
			     const uint8_t* data, uint32_t size)
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
	if (uptime > ((uint32_t)100*365*24*3600*1000)) {
		LOG_ERR("Uptime is more than 100 years.");
		uptime = uptime % ((uint32_t)100*365*24*3600*1000);
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

		if (k_mutex_lock(&ble_receive_buffer_mutex, K_MSEC(10)) == 0) {

			if ((ble_received_count + event->len) <= CONFIG_DIAGNOSTICS_RECEIVE_BUFFER_LENGTH) {
				memcpy(&ble_receive_buffer[ble_received_count], event->buf, event->len);
			} else {
				LOG_ERR("BLE receive data overflow.");
			}

			// TODO - Replace with task notification indicating new data?
			atomic_set_bit(&ble_has_new_data, 0);
			
			k_mutex_unlock(&ble_receive_buffer_mutex);
		}


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
