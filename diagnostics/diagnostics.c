/*
 * Copyright (c) 2022 Nofence AS
 */

#include "diagnostics.h"
#include "diagnostics_types.h"
#include "passthrough.h"

#if CONFIG_DIAGNOSTICS_TEXT_PARSER
#include "parser.h"
#endif

#include "diagnostics_events.h"
#include "msg_data_event.h"
#include "ble_data_event.h"
#include "ble_conn_event.h"

#include <zephyr.h>
#include <logging/log.h>

#include <stdio.h>
#include <stdlib.h>

#define LOG_MODULE_NAME diagnostics
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_DIAGNOSTICS_LOG_LEVEL);

/* Maximum supported uptime */
#define DIAGNOSTICS_MAX_UPTIME_MS 100*365*24*3600*1000

/* Id variable for diagnostics handler thread. */
extern const k_tid_t diag_handler_id; 

/* BLE - Connection is signalled through event manager, and state is stored 
 *       in an atomic variable.
 *       Incoming data comes through event manager. Data is copied over into 
 *       a local buffer, and a semaphore is given to indicate data available.
 *       Mutex is used to protect from concurrent usage of the receive buffer.
 *       Outgoing data is sent through the event manager by building a new 
 *       event and submitting as needed. 
 */
K_MUTEX_DEFINE(ble_receive_buffer_mutex);
K_SEM_DEFINE(ble_has_new_data_sem, 0, 1);
atomic_t ble_is_connected = ATOMIC_INIT(0);
uint8_t ble_receive_buffer[CONFIG_DIAGNOSTICS_RECEIVE_BUFFER_LENGTH];
uint32_t ble_received_count = 0;

/* RTT - Incoming data is handled by thread that will poll the RTT API for 
 *       new data.
 *       Outgoing data is sent as needed, but protected by a mutex to avoid
 *       concurrent usage.
 */
K_MUTEX_DEFINE(rtt_send_mutex);
K_TIMER_DEFINE(rtt_activity_timer, NULL, NULL);
static uint8_t rtt_up_data_buffer[CONFIG_DIAGNOSTICS_RTT_BUFFER_SIZE];
static uint8_t rtt_down_data_buffer[CONFIG_DIAGNOSTICS_RTT_BUFFER_SIZE];
uint8_t rtt_receive_buffer[CONFIG_DIAGNOSTICS_RECEIVE_BUFFER_LENGTH];
uint32_t rtt_received_count = 0;

/**
 * @brief Restart activity timer for RTT. 
 */
static void diagnostics_rtt_set_active(void)
{
	k_timer_start(&rtt_activity_timer, 
		      K_MSEC(CONFIG_DIAGNOSTICS_RTT_TIMEOUT_MS), 
		      K_NO_WAIT);
}

/**
 * @brief Checks whether RTT is assumed to be active. This is based on an
 *        activity timer timing out after a certain period of idling. 
 * 
 * @return True if RTT activity timer is not timed out. False otherwise. 
 */
static bool diagnostics_rtt_is_active(void)
{
	return (k_timer_remaining_ticks(&rtt_activity_timer) != 0);
}

/**
 * @brief Sends data to RTT interface. 
 * 
 * @param[in] data Data buffer to send.
 * @param[in] size Data size to send. 
 */
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

/**
 * @brief Sends data to BLE interface. 
 * 
 * @param[in] data Data buffer to send.
 * @param[in] size Data size to send. 
 */
static void diagnostics_send_ble(const uint8_t* data, uint32_t size)
{
	struct msg_data_event *msg = new_msg_data_event(size);
	memcpy(msg->dyndata.data, data, size);
	EVENT_SUBMIT(msg);
}

/**
 * @brief Sends data to specified interface. 
 * 
 * @param[in] interface Interface to send data to. 
 * @param[in] data Data buffer to send.
 * @param[in] size Data size to send. 
 */
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

#if CONFIG_DIAGNOSTICS_TEXT_PARSER
	struct parser_action actions = {
		.send_resp = diagnostics_send,
		.thru_enable = passthrough_enable
	};
	parser_init(&actions);
#endif

	return 0;
}

/**
 * @brief Processes input coming from specified interface.
 *        Input data is sent to passthrough if enabled. Otherwise it is 
 *        sent to the parser. 
 * 
 * @param[in] interface Interface which input data belongs to. 
 * @param[in] data Input data buffer.
 * @param[in] size Input data size. 
 * 
 * @return Number of bytes in buffer that was processed. 
 */
static uint32_t diagnostics_process_input(enum diagnostics_interface interface, 
					uint8_t* data, uint32_t size)
{
	uint32_t bytes_parsed = 0;
	enum diagnostics_interface passthrough_interface = DIAGNOSTICS_NONE;
	passthrough_get_enabled_interface(&passthrough_interface);

	if (passthrough_interface != DIAGNOSTICS_NONE) {
		bytes_parsed = passthrough_write_data(data, size);
	} else {
#if CONFIG_DIAGNOSTICS_TEXT_PARSER
		bytes_parsed = parser_handle(interface, data, size);
#else
		/* Echo for now */
		bytes_parsed = size;
		LOG_ERR("Trying to send diag resp");
		diagnostics_send(interface, data, size);
#endif
	}
	return bytes_parsed;
}

/**
 * @brief Consumes specified amount of data from start of buffer by moving 
 *        following data to index 0. The resulting number of bytes remaining 
 *        is returned. 
 * 
 * @param[inout] buffer Buffer where data should be consumed. 
 * @param[in] consumed Number of bytes to consume. 
 * @param[in] total_bytes Total number of bytes in buffer. 
 * 
 * @return Number of bytes remaining in buffer after consuming. 
 */
static uint32_t diagnostics_buffer_consume(uint8_t* buffer, 
					   uint32_t consumed, 
					   uint32_t total_bytes)
{
	/* Move unparsed data to start of buffer */
	for (uint32_t i = 0; i < (total_bytes-consumed); i++)
	{
		buffer[i] = buffer[consumed+i];
	}

	return total_bytes - consumed;
}

/**
 * @brief Checks for incoming data on RTT. Forwards to 
 *        correct diagnostics interface channel.
 * 
 * @param[out] got_data Notifies caller that there was data available.
 * 
 */
static void diagnostics_handle_rtt_input(bool* got_data)
{
	if (!SEGGER_RTT_HASDATA(CONFIG_DIAGNOSTICS_RTT_DOWN_CHANNEL_INDEX)) {
		/* No new data */
		return;
	}
	diagnostics_rtt_set_active();

	size_t remaining_space = 
		CONFIG_DIAGNOSTICS_RECEIVE_BUFFER_LENGTH - rtt_received_count;
	size_t bytes_read = SEGGER_RTT_Read(
		CONFIG_DIAGNOSTICS_RTT_DOWN_CHANNEL_INDEX, 
		&rtt_receive_buffer[rtt_received_count], 
		remaining_space);

	if (bytes_read > 0) {
		*got_data = true;

		rtt_received_count += bytes_read;
		uint32_t parsed_bytes = diagnostics_process_input(
							DIAGNOSTICS_RTT, 
							rtt_receive_buffer, 
							rtt_received_count);
		
		if (parsed_bytes > 0)
		{
			rtt_received_count = diagnostics_buffer_consume(
							rtt_receive_buffer, 
							parsed_bytes, 
							rtt_received_count);
		}
	}
}

/**
 * @brief Checks for incoming data on BLE. Forwards to 
 *        correct diagnostics interface channel.
 * 
 * @param[out] got_data Notifies caller that there was data available.
 * 
 */
static void diagnostics_handle_ble_input(bool* got_data)
{
	if (k_sem_take(&ble_has_new_data_sem, K_NO_WAIT) != 0) {
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

		k_mutex_unlock(&ble_receive_buffer_mutex);
	}
}

/**
 * @brief Checks for incoming data on passthrough channel. Forwards to 
 *        correct diagnostics interface channel.
 * 
 * @param[out] got_data Notifies caller that there was data available.
 * 
 */
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

/**
 * @brief Diagnostics data handler function. Run as a thread. 
 */
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

K_THREAD_DEFINE(diag_handler_id, 
		CONFIG_DIAGNOSTICS_STACK_SIZE, 
		diagnostics_handler, NULL, NULL, NULL,
		CONFIG_DIAGNOSTICS_THREAD_PRIORITY, 0, K_TICKS_FOREVER);



/**
 * @brief Logs an entry of data. This is only sent to RTT. 
 * 
 * @param[in] severity Severity of the log entry. Adjusts color in logging. 
 * @param[in] header Header provides a context for the following message. 
 * @param[in] msg Message for the log entry. 
 * 
 */
static void diagnostics_log(enum diagnostics_severity severity, 
						    char* header, char* msg)
{
	uint32_t used_size = 0;
	uint32_t uptime = k_uptime_get_32();
	if (uptime > ((uint32_t)DIAGNOSTICS_MAX_UPTIME_MS)) {
		LOG_ERR("Uptime is more than 100 years.");
		uptime = uptime % ((uint32_t)DIAGNOSTICS_MAX_UPTIME_MS);
	}
	char uptime_str[13+2];
	used_size = snprintf(uptime_str, 13+2, "%u.%03u", 
					       uptime/1000, 
					       uptime%1000);
	if ((used_size < 0) || (used_size >= sizeof(uptime_str))) {
		LOG_ERR("Uptime encoding error.");
		return;
	}

	/* Color code + bracket pair + max timestamp + color code + 
	   header_size + ": " + color code + msg_size + linefeed + 
	   color code */
	uint32_t bufsiz = 7 + 2 + 13 + 2 + 7 + 
			  strlen(header) + 2 + 7 + strlen(msg) + 2 + 
			  7;
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
					  RTT_CTRL_TEXT_BRIGHT_GREEN, 
					  uptime_str, 
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
 * @brief Main event handler function. Listens to BLE connection state and 
 *        handles received data. 
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

			if ((ble_received_count + event->len) <= 
			    CONFIG_DIAGNOSTICS_RECEIVE_BUFFER_LENGTH) {
				memcpy(&ble_receive_buffer[ble_received_count],
				       event->buf, 
				       event->len);
			} else {
				LOG_ERR("BLE receive data overflow.");
			}

			k_sem_give(&ble_has_new_data_sem);
			
			k_mutex_unlock(&ble_receive_buffer_mutex);
		}


		return false;
	}

	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, ble_conn_event);
EVENT_SUBSCRIBE(MODULE, ble_data_event);

#if CONFIG_DIAGNOSTICS_PROFILE_EVENTS

/**
 * @brief Event manager trace init. Called when initializing the event manager.
 *
 * @return Return code 0 is OK. 
 */
int event_manager_trace_event_init(void)
{
	diagnostics_log(DIAGNOSTICS_INFO, 
			"EVENT_MANAGER_INIT", 
			"Initializing");
	return 0;
}

/**
 * @brief Helper function to build string from specified event. 
 *
 * @param[in] eh Event to build string from. 
 * @param[out] buffer Buffer to populate with built string.
 * @param[in] size Maximum size of buffer. 
 */
static void diagnostics_build_event_string(const struct event_header *eh, 
					   uint8_t* buffer, 
					   uint32_t size)
{
	/* Must have space for event name, 
	parantheses, space and null-termination. */
	if (size < (strlen(eh->type_id->name) + 4)) {
		LOG_ERR("Too small buffer for diagnostics logging of event.");
	}

	uint32_t length = snprintf(buffer, size, "(%s) ", eh->type_id->name);
	eh->type_id->log_event(eh, &buffer[length], size-length);
}

/**
 * @brief Event manager trace execution override. Called when starting and
 *        stopping execution of subscribers. 
 *
 * @param[in] eh Event that is executed. 
 * @param[in] is_start True when starting, false when stopping. 
 */
void event_manager_trace_event_execution(const struct event_header *eh,
					 bool is_start)
{
	char buf[100];
	diagnostics_build_event_string(eh, buf, sizeof(buf));
	diagnostics_log(DIAGNOSTICS_INFO, 
			is_start ? "EVENT_MANAGER_EXEC_START" : 
				   "EVENT_MANAGER_EXEC_STOP", 
			buf);
}

/**
 * @brief Event manager trace submission override. Called when event was
 *        submitted. 
 *
 * @param[in] eh Event that is submitted. 
 * @param[in] trace_info Trace info. 
 */
void event_manager_trace_event_submission(const struct event_header *eh,
				const void *trace_info)
{
	char buf[100];
	diagnostics_build_event_string(eh, buf, sizeof(buf));
	diagnostics_log(DIAGNOSTICS_INFO, "EVENT_MANAGER_SUBMIT", buf);
}

#endif
