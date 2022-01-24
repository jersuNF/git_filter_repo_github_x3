/*
 * Copyright (c) 2022 Nofence AS
 */

#include <zephyr.h>
#include "diagnostics_events.h"
#include "diagnostics.h"
#include <logging/log.h>
#include <sys/ring_buffer.h>
#include <stdio.h>
#include <stdlib.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <drivers/eeprom.h>
#include <drivers/sensor.h>

#include "nf_version.h"
#include "version.h"

#include "msg_data_event.h"

#include "ble_ctrl_event.h"
#include "ble_data_event.h"
#include "ble_conn_event.h"

#define LOG_MODULE_NAME diagnostics
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_DIAGNOSTICS_LOG_LEVEL);

#define MAX_ARG_LEN	20

K_MUTEX_DEFINE(rtt_send_mutex);
K_TIMER_DEFINE(rtt_activity_timer, NULL, NULL);

#define THREAD_PRIORITY 7

extern const k_tid_t diag_handler_id; 

static uint8_t diagnostics_up_data_buffer[CONFIG_DIAGNOSTICS_RTT_BUFFER_SIZE];
static uint8_t diagnostics_down_data_buffer[CONFIG_DIAGNOSTICS_RTT_BUFFER_SIZE];

atomic_t ble_is_connected = ATOMIC_INIT(0);

K_MUTEX_DEFINE(ble_receive_buffer_mutex);
uint8_t ble_receive_buffer[CONFIG_DIAGNOSTICS_RECEIVE_BUFFER_LENGTH];
uint32_t ble_received_count = 0;
atomic_t ble_has_new_data = ATOMIC_INIT(0);

uint8_t rtt_receive_buffer[CONFIG_DIAGNOSTICS_RECEIVE_BUFFER_LENGTH];
uint32_t rtt_received_count = 0;

atomic_t passthrough_tx_in_progress = ATOMIC_INIT(0);
uint8_t passthrough_tx_buffer[CONFIG_DIAGNOSTICS_PASSTHROUGH_BUFFER_SIZE];
uint32_t passthrough_tx_count = 0;

/* Passthrough variables for UART (GNSS and modem) */
atomic_t passthrough_enabled = ATOMIC_INIT(0);
enum diagnostics_interface passthrough_interface = DIAGNOSTICS_NONE;
const struct device *passthrough_device = NULL;
struct ring_buf passthrough_ring_buf;
uint8_t* passthrough_buffer = NULL;

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

static int parser_run(enum diagnostics_interface interface, char* cmd, uint32_t length);
static uint32_t parser_get_next(char* arg, char** next_arg);
static bool parser_arg_is_matching(char* arg, char* sub, uint32_t sub_len);
static int parser_test(enum diagnostics_interface interface, char* arg);

static const struct device *gpio0_dev;
static const struct device *gpio1_dev;

int diagnostics_module_init()
{
	SEGGER_RTT_Init();

	if (SEGGER_RTT_ConfigUpBuffer(CONFIG_DIAGNOSTICS_RTT_UP_CHANNEL_INDEX, "DIAG_UP", diagnostics_up_data_buffer, CONFIG_DIAGNOSTICS_RTT_BUFFER_SIZE, SEGGER_RTT_MODE_NO_BLOCK_SKIP) != 0) {
		LOG_ERR("Diagnostics module failed to setup RTT up channel.");
	}
	if (SEGGER_RTT_ConfigDownBuffer(CONFIG_DIAGNOSTICS_RTT_DOWN_CHANNEL_INDEX, "DIAG_DOWN", diagnostics_down_data_buffer, CONFIG_DIAGNOSTICS_RTT_BUFFER_SIZE, SEGGER_RTT_MODE_NO_BLOCK_SKIP) != 0) {
		LOG_ERR("Diagnostics module failed to setup RTT down channel.");
	}
	
	gpio0_dev = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
	gpio1_dev = device_get_binding(DT_LABEL(DT_NODELABEL(gpio1)));

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

static uint32_t diagnostics_buffer_consume(uint8_t* buffer, uint32_t consumed, uint32_t total_bytes)
{
	/* Move unparsed data to start of buffer */
	for (uint32_t i = 0; i < (total_bytes-consumed); i++)
	{
		buffer[i] = buffer[consumed+i];
	}

	return total_bytes - consumed;
}

static void diagnostics_handler(void)
{
	while (true) {
		bool processed_data = false;
		if (SEGGER_RTT_HASDATA(CONFIG_DIAGNOSTICS_RTT_DOWN_CHANNEL_INDEX)) {
			diagnostics_rtt_set_active();

			size_t bytes_read = SEGGER_RTT_Read(CONFIG_DIAGNOSTICS_RTT_DOWN_CHANNEL_INDEX, 
												&rtt_receive_buffer[rtt_received_count], CONFIG_DIAGNOSTICS_RECEIVE_BUFFER_LENGTH - rtt_received_count);
		
			if (bytes_read > 0) {
				rtt_received_count += bytes_read;
				uint32_t parsed_bytes = diagnostics_parse_input(DIAGNOSTICS_RTT, rtt_receive_buffer, rtt_received_count);
				
				if (parsed_bytes > 0)
				{
					rtt_received_count = diagnostics_buffer_consume(rtt_receive_buffer, parsed_bytes, rtt_received_count);
				}
			}
		}
		if (atomic_test_bit(&ble_has_new_data, 0)) {
			if (k_mutex_lock(&ble_receive_buffer_mutex, K_MSEC(10)) == 0) {

				uint32_t parsed_bytes = diagnostics_parse_input(DIAGNOSTICS_BLE, 
													ble_receive_buffer, ble_received_count);

				if (parsed_bytes > 0)
				{
					ble_received_count = diagnostics_buffer_consume(ble_receive_buffer, parsed_bytes, ble_received_count);
				}

				atomic_clear_bit(&ble_has_new_data, 0);

				k_mutex_unlock(&ble_receive_buffer_mutex);
			}
		}

		if (atomic_test_bit(&passthrough_enabled, 0)) {
			if (!ring_buf_is_empty(&passthrough_ring_buf)) {
				uint8_t* pass_data;
				uint32_t pass_size = ring_buf_get_claim(&passthrough_ring_buf, &pass_data, CONFIG_DIAGNOSTICS_PASSTHROUGH_BUFFER_SIZE);

				diagnostics_send(passthrough_interface, pass_data, pass_size);

				int err = ring_buf_get_finish(&passthrough_ring_buf, pass_size);
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

static bool diagnostics_find_newline_offset(char* str, uint32_t length, uint32_t* newline_offset) {
	uint32_t offset = 0;
	while (str[offset] != '\n') {
		offset++;
		if (offset == length) {
			return false;
		}
	}
	*newline_offset = offset;
	return true;
}

static uint32_t diagnostics_parse_input(enum diagnostics_interface interface, 
					uint8_t* data, uint32_t size)
{
	uint32_t bytes_parsed = 0;
	if (atomic_test_bit(&passthrough_enabled, 0) && (interface == passthrough_interface)) {
		if (!atomic_test_bit(&passthrough_tx_in_progress, 0)) {
			//bytes_parsed = min(DIAGNOSTICS_PASSTHROUGH_BUFFER_SIZE, size);
			//memcpy(passthrough_tx_buffer, data, bytes_parsed;
			
			//uart_irq_tx_enable(passthrough_device);
		}
	} else {
		uint32_t newline_loc = 0;
		if (diagnostics_find_newline_offset(data, size, &newline_loc)) {
			bytes_parsed = newline_loc+1;
			uint32_t cmd_length = newline_loc;
			while ((cmd_length >= 1) && (data[cmd_length-1] == '\r'))
			{
				cmd_length--;
			}

			data[cmd_length] = '\0';
			parser_run(interface, data, cmd_length);
		}
	}
	return bytes_parsed;
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

static uint32_t parser_get_next(char* arg, char** next_arg)
{
	if (arg == NULL) {
		*next_arg = NULL;
		return 0;
	}

	uint32_t length = 0;
	while ((arg[length] != '\0') && (arg[length] != ' ')) {
		if (length >= MAX_ARG_LEN) {
			*next_arg = NULL;
			return 0;
		}

		length++;
	}
	/* Next argument is NULL when there are not more arguments */
	*next_arg = arg[length] ? &arg[length + 1] : NULL;
	return length;
}

static bool parser_arg_is_matching(char* arg, char* sub, uint32_t sub_len)
{
	return ((strlen(arg) == sub_len) && (strncmp(sub, arg, sub_len) == 0));
}

static int parser_test(enum diagnostics_interface interface, char* arg)
{
	char* next_arg;
	uint32_t sub_arg_len = parser_get_next(arg, &next_arg);
	if (sub_arg_len == 0) {
		const char* noargs = "Test requires an argument, e.g. test lis2dw\r\n";
		diagnostics_send(interface, noargs, strlen(noargs));
		return 0;
	}

	if (parser_arg_is_matching("lis2dw", arg, sub_arg_len)) {
		//const struct device *sensor = device_get_binding(DT_LABEL(DT_INST(0, st_lis2dh)));
		const struct device *sensor = device_get_binding(DT_LABEL(DT_NODELABEL(movement)));

		if (sensor == NULL) {
			const char* nodev = "LIS2DH - No device found.\r\n";
			diagnostics_send(interface, nodev, strlen(nodev));
		}
		if (!device_is_ready(sensor)) {
			const char* readydev = "LIS2DH - Device is NOT ready.\r\n";
			diagnostics_send(interface, readydev, strlen(readydev));
		} else {
			const char* readydev = "LIS2DH - Device is ready.\r\n";
			diagnostics_send(interface, readydev, strlen(readydev));
		}
	} else if (parser_arg_is_matching("eeprom", arg, sub_arg_len)) {
		const struct device *eeprom_dev = device_get_binding(DT_LABEL(DT_NODELABEL(eeprom0)));
		
		if (eeprom_dev == NULL) {
			const char* nodev = "EEPROM - No device found.\r\n";
			diagnostics_send(interface, nodev, strlen(nodev));
		}
		if (!device_is_ready(eeprom_dev)) {
			const char* readydev = "EEPROM - Device is NOT ready.\r\n";
			diagnostics_send(interface, readydev, strlen(readydev));
		} else {
			const char* readydev = "EEPROM - Device is ready.\r\n";
			diagnostics_send(interface, readydev, strlen(readydev));
		}

		uint8_t data = 0;
		if (eeprom_read(eeprom_dev, 0, &data, 1) != 0) {
			const char* readydev = "EEPROM - Failed reading offset 0.\r\n";
			diagnostics_send(interface, readydev, strlen(readydev));
		}
		char buf[10];
		itoa(data, buf, 10);
		diagnostics_send(interface, buf, strlen(buf));
		diagnostics_send(interface, "\r\n", strlen("\r\n"));

		data = 0xFF;
		if (eeprom_write(eeprom_dev, 0, &data, 1) != 0) {
			const char* readydev = "EEPROM - Failed writing offset 0.\r\n";
			diagnostics_send(interface, readydev, strlen(readydev));
		}

		data = 0;
		if (eeprom_read(eeprom_dev, 0, &data, 1) != 0) {
			const char* readydev = "EEPROM - Failed reading offset 0.\r\n";
			diagnostics_send(interface, readydev, strlen(readydev));
		}

		itoa(data, buf, 10);
		diagnostics_send(interface, buf, strlen(buf));
		diagnostics_send(interface, "\r\n", strlen("\r\n"));
	} else if (parser_arg_is_matching("bme280", arg, sub_arg_len)) {
		const struct device *bme_dev = device_get_binding(DT_LABEL(DT_NODELABEL(environment)));
		
		if (bme_dev == NULL) {
			const char* nodev = "BME280 - No device found.\r\n";
			diagnostics_send(interface, nodev, strlen(nodev));
		}
		if (!device_is_ready(bme_dev)) {
			const char* readydev = "BME280 - Device is NOT ready.\r\n";
			diagnostics_send(interface, readydev, strlen(readydev));
		} else {
			const char* readydev = "BME280 - Device is ready.\r\n";
			diagnostics_send(interface, readydev, strlen(readydev));
		}

		struct sensor_value temp, press, humidity;

		sensor_sample_fetch(bme_dev);
		sensor_channel_get(bme_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(bme_dev, SENSOR_CHAN_PRESS, &press);
		sensor_channel_get(bme_dev, SENSOR_CHAN_HUMIDITY, &humidity);

		char buf[100];
		snprintf(buf, 100, "temp: %d.%06d; press: %d.%06d; humidity: %d.%06d\r\n",
		      temp.val1, temp.val2, press.val1, press.val2,
		      humidity.val1, humidity.val2);
		diagnostics_send(interface, buf, strlen(buf));
	} else {
		const char* readydev = "Unknown test parameter\r\n";
		diagnostics_send(interface, readydev, strlen(readydev));
	}

	return 0;
}

static void passthrough_flush(const struct device *uart_dev)
{
	uint8_t c;

	while (uart_fifo_read(uart_dev, &c, 1) > 0) {
		continue;
	}
	
}

static void passthrough_uart_isr(const struct device *uart_dev,
				 		  		 void *user_data)
{
	uint32_t total_received = 0;
	uint32_t partial_size = 0;
	uint8_t* uart_data;

	uart_irq_update(uart_dev);

	while (uart_irq_update(uart_dev) &&
	        uart_irq_rx_ready(uart_dev)) {
		
		/*if (uart_irq_tx_ready(h4_dev)) {
			bytes = uart_fifo_fill(h4_dev, &tx.type, 1);
			uart_irq_tx_disable(h4_dev);
		}*/

		if (partial_size == 0) {
			partial_size = 
					ring_buf_put_claim(&passthrough_ring_buf, 
									   &uart_data, 
									   CONFIG_DIAGNOSTICS_PASSTHROUGH_BUFFER_SIZE);
		}

		if (partial_size == 0) {
			passthrough_flush(uart_dev);
			break;
		}

		uint32_t received = uart_fifo_read(uart_dev, uart_data, partial_size);
		if (received <= 0) {
			continue;
		}

		uart_data += received;
		total_received += received;
		partial_size -= received;
	}
		
	int ret = ring_buf_put_finish(&passthrough_ring_buf, total_received);
	__ASSERT_NO_MSG(ret == 0);
}

static int passthrough_initialize(const struct device* dev)
{
	if (passthrough_device != NULL) {
		uart_irq_rx_disable(passthrough_device);
		uart_irq_tx_disable(passthrough_device);
		passthrough_device = NULL;
	}
	
	if (passthrough_buffer == NULL)
	{
		passthrough_buffer = 
			k_malloc(CONFIG_DIAGNOSTICS_PASSTHROUGH_BUFFER_SIZE);

		if (passthrough_buffer == NULL) {
			return -ENOBUFS;
		}
    	ring_buf_init(&passthrough_ring_buf, 
					  CONFIG_DIAGNOSTICS_PASSTHROUGH_BUFFER_SIZE, 
					  passthrough_buffer);
	}

	passthrough_device = dev;
	
	if (passthrough_device == NULL) {
		const char* readydev = "Passthrough UART not ready\r\n";
		diagnostics_send(passthrough_interface, readydev, strlen(readydev));
		return -EIO;
	}
	if (!device_is_ready(passthrough_device)) {
		const char* readydev = "Passthrough UART not ready\r\n";
		diagnostics_send(passthrough_interface, readydev, strlen(readydev));
		return -EIO;
	}
	
	uart_irq_rx_disable(passthrough_device);
	uart_irq_tx_disable(passthrough_device);

	passthrough_flush(passthrough_device);

	uart_irq_callback_set(passthrough_device, passthrough_uart_isr);
	uart_irq_rx_enable(passthrough_device);

	const char* readydev = "Passthrough Starting...\r\n";
	diagnostics_send(passthrough_interface, readydev, strlen(readydev));

	atomic_set_bit(&passthrough_enabled, 0);

	return 0;
}

static int parser_gnss(enum diagnostics_interface interface, char* arg)
{
	passthrough_interface = interface;
	const struct device *gnss_device = device_get_binding(DT_LABEL(DT_NODELABEL(gnss_uart)));

	return passthrough_initialize(gnss_device);
}

static int parser_modem(enum diagnostics_interface interface, char* arg)
{
	passthrough_interface = interface;
	const struct device *modem_device = device_get_binding(DT_LABEL(DT_NODELABEL(modem_uart)));

	return passthrough_initialize(modem_device);
}

static int parser_gpio(enum diagnostics_interface interface, char* arg)
{
	char* next_arg;
	uint32_t sub_arg_len = parser_get_next(arg, &next_arg);
	if (sub_arg_len == 0) {
		const char* noargs = "GPIO requires arguments: gpio <port> <pin> <z/0/1/in/inpu/inpd/?> <val>\r\n";
		diagnostics_send(interface, noargs, strlen(noargs));
		return 0;
	}

	/* Resolve port device */
	struct device *gpio_dev = gpio0_dev;
	if ((sub_arg_len == 1) && (arg[0] == '0')) {
		gpio_dev = gpio0_dev;
	} else if ((sub_arg_len == 1) && (arg[0] == '1')) {
		gpio_dev = gpio1_dev;
	} else {
		const char* invargs = "Invalid port for GPIO\r\n";
		diagnostics_send(interface, invargs, strlen(invargs));
		return 0;
	}

	/* Resolve pin */
	arg = next_arg;
	sub_arg_len = parser_get_next(arg, &next_arg);

	int pin = atoi(arg);
	if ((pin < 0) || (pin >= 32)) {
		const char* invargs = "Invalid pin for GPIO\r\n";
		diagnostics_send(interface, invargs, strlen(invargs));
		return 0;
	}

	/* Get operation argument location and size, store for later use */
	char* operation_arg = next_arg;
	uint32_t operation_arg_len = parser_get_next(operation_arg, &next_arg);

	/* Perform operation with given arguments */
	int err = -EPERM;
	if (parser_arg_is_matching("z", operation_arg, operation_arg_len)) {
		err = gpio_pin_configure(gpio_dev, pin, GPIO_DISCONNECTED);
	} else if (parser_arg_is_matching("in", operation_arg, operation_arg_len)) {
		err = gpio_pin_configure(gpio_dev, pin, GPIO_INPUT);
	} else if (parser_arg_is_matching("inpu", operation_arg, operation_arg_len)) {
		uint32_t in_flags = GPIO_INPUT | GPIO_PULL_UP;
		err = gpio_pin_configure(gpio_dev, pin, in_flags);
	} else if (parser_arg_is_matching("inpd", operation_arg, operation_arg_len)) {
		uint32_t in_flags = GPIO_INPUT | GPIO_PULL_DOWN;
		err = gpio_pin_configure(gpio_dev, pin, in_flags);
	} else if (parser_arg_is_matching("0", operation_arg, operation_arg_len)) {
		uint32_t out_flags = GPIO_OUTPUT_LOW;
		err = gpio_pin_configure(gpio_dev, pin, out_flags);
	} else if (parser_arg_is_matching("1", operation_arg, operation_arg_len)) {
		uint32_t out_flags = GPIO_OUTPUT_HIGH;
		err = gpio_pin_configure(gpio_dev, pin, out_flags);
	} else if (parser_arg_is_matching("?", operation_arg, operation_arg_len)) {
		uint32_t values = 0;
		err = gpio_port_get_raw(gpio_dev, &values);
		if (err == 0) {
			const char* invargs = "GPIO pin values is ";
			diagnostics_send(interface, invargs, strlen(invargs));
			
			if ((values & (1<<pin)) != 0) {
				diagnostics_send(interface, "1", strlen("1"));
			} else {
				diagnostics_send(interface, "0", strlen("0"));
			}
			
			diagnostics_send(interface, "\r\n", strlen("\r\n"));
		}
	} else {
		const char* invargs = "Invalid operation for GPIO\r\n";
		diagnostics_send(interface, invargs, strlen(invargs));
		return 0;
	}

	if (err == 0) {
		const char* okrsp = "GPIO ok\r\n";
		diagnostics_send(interface, okrsp, strlen(okrsp));
	} else {
		const char* nokrsp = "GPIO failed\r\n";
		diagnostics_send(interface, nokrsp, strlen(nokrsp));
	}

	return err;
}

static int parser_help(enum diagnostics_interface interface, char* arg)
{
	const char* help = "Sorry.. No help yet.\r\n";
	diagnostics_send(interface, help, strlen(help));

	return 0;
}

static int parser_run(enum diagnostics_interface interface, char* cmd, uint32_t length)
{
	char* next_arg;
	uint32_t cmd_len = parser_get_next(cmd, &next_arg);
	if (cmd_len == 0) {
		const char* info = "-= Nofence collar diagnostics at your service =-\r\nType help for more information.\r\n";
		diagnostics_send(interface, info, strlen(info));
		diagnostics_send(interface, "Zephyr version is ", strlen("Zephyr version is "));
		diagnostics_send(interface, KERNEL_VERSION_STRING, strlen(KERNEL_VERSION_STRING));
		diagnostics_send(interface, "\r\n", strlen("\r\n"));
		return 0;
	}

	if (parser_arg_is_matching("help", cmd, cmd_len)) {
		parser_help(interface, next_arg);
	} else if (parser_arg_is_matching("test", cmd, cmd_len)) {
		parser_test(interface, next_arg);
	} else if (parser_arg_is_matching("gnss", cmd, cmd_len)) {
		parser_gnss(interface, next_arg);
	} else if (parser_arg_is_matching("modem", cmd, cmd_len)) {
		parser_modem(interface, next_arg);
	} else if (parser_arg_is_matching("gpio", cmd, cmd_len)) {
		parser_gpio(interface, next_arg);
	} else {
		diagnostics_send(interface, "Dunno\r\n", strlen("Dunno\r\n"));
	}

	return 0;
}
