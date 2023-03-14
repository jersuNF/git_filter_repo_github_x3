/*
 * Copyright (c) 2022 Nofence AS
 */

#include "parser.h"
#include "nf_version.h"
#include "version.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/eeprom.h>
#include <drivers/sensor.h>
#include <drivers/flash.h>

#define MAX_ARG_LEN 20

static struct parser_action parser_actions;

static const struct device *gpio0_dev;
static const struct device *gpio1_dev;

static const struct device *modem_device;
static const struct device *gnss_device;

int parser_init(struct parser_action *actions)
{
	gpio0_dev = device_get_binding(DT_LABEL(DT_NODELABEL(gpio0)));
	gpio1_dev = device_get_binding(DT_LABEL(DT_NODELABEL(gpio1)));

	modem_device = device_get_binding(DT_LABEL(DT_NODELABEL(modem_uart)));
	gnss_device = device_get_binding(DT_LABEL(DT_NODELABEL(gnss_uart)));

	if (actions != NULL) {
		parser_actions = *actions;
	}

	return 0;
}

/**
 * @brief Finds offset of newline character in provided string. 
 * 
 * @param[in] str String to search. 
 * @param[in] length Length of string. 
 * @param[out] length Offset of newline character. 
 * 
 * @return True if newline was found, false otherwise. 
 */
static bool parser_find_newline_offset(char *str, uint32_t length, uint32_t *newline_offset)
{
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

/**
 * @brief Checks if argument string and string of defined length is matching. 
 * 
 * @param[in] arg Argument string to match. 
 * @param[in] sub Substring to match. 
 * @param[in] sub_len Length of substring. 
 * 
 * @return True if matching, false otherwise. 
 */
static bool parser_arg_is_matching(char *arg, char *sub, uint32_t sub_len)
{
	return ((strlen(arg) == sub_len) && (strncmp(sub, arg, sub_len) == 0));
}

/**
 * @brief Gets index of next argument, split on space character. 
 * 
 * @param[in] arg Argument string to search. 
 * @param[out] next_arg Pointer to next argument within arg. NULL if none. 
 * 
 * @return Index of next argument. 0 if no more arguments. 
 */
static uint32_t parser_get_next(char *arg, char **next_arg)
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

/**
 * @brief Implementation of test commands
 * 
 * @param[in] interface Interface that sent command. 
 * @param[in] arg String with arguments for command. 
 * 
 * @return 0 if ok, error code otherwise. 
 */
static int parser_test(enum diagnostics_interface interface, char *arg)
{
	char *next_arg;
	uint32_t sub_arg_len = parser_get_next(arg, &next_arg);
	if (sub_arg_len == 0) {
		const char *noargs = "Specify test: lis2dw, eeprom, bme280, flash\r\n";
		parser_actions.send_resp(interface, noargs, strlen(noargs));
		return 0;
	}

	if (parser_arg_is_matching("lis2dw", arg, sub_arg_len)) {
		const struct device *sensor =
			device_get_binding(DT_LABEL(DT_NODELABEL(movement_sensor)));

		if (sensor == NULL) {
			const char *nodev = "LIS2DH - No device found.\r\n";
			parser_actions.send_resp(interface, nodev, strlen(nodev));
		}
		if (!device_is_ready(sensor)) {
			const char *readydev = "LIS2DH - Device is NOT ready.\r\n";
			parser_actions.send_resp(interface, readydev, strlen(readydev));
		} else {
			const char *readydev = "LIS2DH - Device is ready.\r\n";
			parser_actions.send_resp(interface, readydev, strlen(readydev));
		}
	} else if (parser_arg_is_matching("eeprom", arg, sub_arg_len)) {
		const struct device *eeprom_dev =
			device_get_binding(DT_LABEL(DT_NODELABEL(eeprom0)));

		if (eeprom_dev == NULL) {
			const char *nodev = "EEPROM - No device found.\r\n";
			parser_actions.send_resp(interface, nodev, strlen(nodev));
		}
		if (!device_is_ready(eeprom_dev)) {
			const char *readydev = "EEPROM - Device is NOT ready.\r\n";
			parser_actions.send_resp(interface, readydev, strlen(readydev));
		} else {
			const char *readydev = "EEPROM - Device is ready.\r\n";
			parser_actions.send_resp(interface, readydev, strlen(readydev));
		}

		uint8_t data = 0;
		if (eeprom_read(eeprom_dev, 0, &data, 1) != 0) {
			const char *readydev = "EEPROM - Failed reading offset 0.\r\n";
			parser_actions.send_resp(interface, readydev, strlen(readydev));
		}
		char buf[10];
		snprintf(buf, 10, "%u", data);
		parser_actions.send_resp(interface, buf, strlen(buf));
		parser_actions.send_resp(interface, "\r\n", strlen("\r\n"));

		data = 0x13;
		if (eeprom_write(eeprom_dev, 0, &data, 1) != 0) {
			const char *readydev = "EEPROM - Failed writing offset 0.\r\n";
			parser_actions.send_resp(interface, readydev, strlen(readydev));
		}

		data = 0;
		if (eeprom_read(eeprom_dev, 0, &data, 1) != 0) {
			const char *readydev = "EEPROM - Failed reading offset 0.\r\n";
			parser_actions.send_resp(interface, readydev, strlen(readydev));
		}

		snprintf(buf, 10, "%u", data);
		parser_actions.send_resp(interface, buf, strlen(buf));
		parser_actions.send_resp(interface, "\r\n", strlen("\r\n"));

		data = 0xFF;
		if (eeprom_write(eeprom_dev, 0, &data, 1) != 0) {
			const char *readydev = "EEPROM - Failed writing offset 0.\r\n";
			parser_actions.send_resp(interface, readydev, strlen(readydev));
		}
	} else if (parser_arg_is_matching("bme280", arg, sub_arg_len)) {
		const struct device *bme_dev =
			device_get_binding(DT_LABEL(DT_NODELABEL(environment_sensor)));

		if (bme_dev == NULL) {
			const char *nodev = "BME280 - No device found.\r\n";
			parser_actions.send_resp(interface, nodev, strlen(nodev));
		}
		if (!device_is_ready(bme_dev)) {
			const char *readydev = "BME280 - Device is NOT ready.\r\n";
			parser_actions.send_resp(interface, readydev, strlen(readydev));
		} else {
			const char *readydev = "BME280 - Device is ready.\r\n";
			parser_actions.send_resp(interface, readydev, strlen(readydev));
		}

		struct sensor_value temp, press, humidity;

		sensor_sample_fetch(bme_dev);
		sensor_channel_get(bme_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(bme_dev, SENSOR_CHAN_PRESS, &press);
		sensor_channel_get(bme_dev, SENSOR_CHAN_HUMIDITY, &humidity);

		char buf[100];
		snprintf(buf, 100, "temp: %d.%06d; press: %d.%06d; humidity: %d.%06d\r\n",
			 temp.val1, temp.val2, press.val1, press.val2, humidity.val1,
			 humidity.val2);
		parser_actions.send_resp(interface, buf, strlen(buf));
	} else if (parser_arg_is_matching("flash", arg, sub_arg_len)) {
		const struct device *flash_dev =
			device_get_binding(DT_LABEL(DT_NODELABEL(mx25u64)));

		if (flash_dev == NULL) {
			const char *nodev = "Flash - No device found.\r\n";
			parser_actions.send_resp(interface, nodev, strlen(nodev));
		}
		if (!device_is_ready(flash_dev)) {
			const char *readydev = "Flash - Device is NOT ready.\r\n";
			parser_actions.send_resp(interface, readydev, strlen(readydev));
			return -1;
		} else {
			const char *readydev = "Flash - Device is ready.\r\n";
			parser_actions.send_resp(interface, readydev, strlen(readydev));
		}

		char buf[10];
		uint8_t test_data[] = { 0, 0 };

		if (flash_read(flash_dev, 0, test_data, 2) != 0) {
			const char *nodev = "Flash - read failed.\r\n";
			parser_actions.send_resp(interface, nodev, strlen(nodev));
		}

		snprintf(buf, 10, "%02X", test_data[0]);
		uint32_t length = strlen(buf);
		buf[length] = ' ';
		snprintf(&buf[length + 1], 10 - length, "%02X", test_data[1]);
		parser_actions.send_resp(interface, "Before write ", strlen("Before write "));
		parser_actions.send_resp(interface, buf, strlen(buf));
		parser_actions.send_resp(interface, "\r\n", strlen("\r\n"));

		test_data[0] = 0x13;
		test_data[1] = 0x37;
		if (flash_write(flash_dev, 0, test_data, 2) != 0) {
			const char *nodev = "Flash - write failed.\r\n";
			parser_actions.send_resp(interface, nodev, strlen(nodev));
		}

		memset(test_data, 0, 2);

		if (flash_read(flash_dev, 0, test_data, 2) != 0) {
			const char *nodev = "Flash - read failed.\r\n";
			parser_actions.send_resp(interface, nodev, strlen(nodev));
		}

		snprintf(buf, 10, "%02X", test_data[0]);
		length = strlen(buf);
		buf[length] = ' ';
		snprintf(&buf[length + 1], 10 - length, "%02X", test_data[1]);
		parser_actions.send_resp(interface, "After write ", strlen("After write "));
		parser_actions.send_resp(interface, buf, strlen(buf));
		parser_actions.send_resp(interface, "\r\n", strlen("\r\n"));

		if (flash_erase(flash_dev, 0, 4096) != 0) {
			const char *nodev = "Flash - erase failed.\r\n";
			parser_actions.send_resp(interface, nodev, strlen(nodev));
		}

		memset(test_data, 0, 2);

		if (flash_read(flash_dev, 0, test_data, 2) != 0) {
			const char *nodev = "Flash - read failed.\r\n";
			parser_actions.send_resp(interface, nodev, strlen(nodev));
		}

		snprintf(buf, 10, "%02X", test_data[0]);
		length = strlen(buf);
		buf[length] = ' ';
		snprintf(&buf[length + 1], 10 - length, "%02X", test_data[1]);
		parser_actions.send_resp(interface, "After erase ", strlen("After erase "));
		parser_actions.send_resp(interface, buf, strlen(buf));
		parser_actions.send_resp(interface, "\r\n", strlen("\r\n"));
	} else {
		const char *readydev = "Unknown test parameter\r\n";
		parser_actions.send_resp(interface, readydev, strlen(readydev));
	}

	return 0;
}

/**
 * @brief Implementation of gnss command. 
 * 
 * @param[in] interface Interface that sent command. 
 * @param[in] arg String with arguments for command. 
 * 
 * @return 0 if ok, error code otherwise. 
 */
static int parser_gnss(enum diagnostics_interface interface, char *arg)
{
	return parser_actions.thru_enable(interface, gnss_device);
}

/**
 * @brief Implementation of modem command. 
 * 
 * @param[in] interface Interface that sent command. 
 * @param[in] arg String with arguments for command. 
 * 
 * @return 0 if ok, error code otherwise. 
 */
static int parser_modem(enum diagnostics_interface interface, char *arg)
{
	/* Enable modem by toggling power pin */
	gpio_pin_configure(gpio0_dev, 2, GPIO_OUTPUT_HIGH);
	k_msleep(1000);
	gpio_pin_configure(gpio0_dev, 2, GPIO_OUTPUT_LOW);

	return parser_actions.thru_enable(interface, modem_device);
}

/**
 * @brief Implementation of gpio command. 
 * 
 * @param[in] interface Interface that sent command. 
 * @param[in] arg String with arguments for command. 
 * 
 * @return 0 if ok, error code otherwise. 
 */
static int parser_gpio(enum diagnostics_interface interface, char *arg)
{
	char *next_arg;
	uint32_t sub_arg_len = parser_get_next(arg, &next_arg);
	if (sub_arg_len == 0) {
		const char *noargs = "GPIO requires arguments: "
				     "gpio <port> <pin> <z/0/1/in/inpu/inpd/?>, "
				     "e.g. gpio 0 8 1\r\n";
		parser_actions.send_resp(interface, noargs, strlen(noargs));
		return 0;
	}

	/* Resolve port device */
	const struct device *gpio_dev = gpio0_dev;
	if ((sub_arg_len == 1) && (arg[0] == '0')) {
		gpio_dev = gpio0_dev;
	} else if ((sub_arg_len == 1) && (arg[0] == '1')) {
		gpio_dev = gpio1_dev;
	} else {
		const char *invargs = "Invalid port for GPIO\r\n";
		parser_actions.send_resp(interface, invargs, strlen(invargs));
		return 0;
	}

	/* Resolve pin */
	arg = next_arg;
	sub_arg_len = parser_get_next(arg, &next_arg);

	int pin = atoi(arg);
	if ((pin < 0) || (pin >= 32)) {
		const char *invargs = "Invalid pin for GPIO\r\n";
		parser_actions.send_resp(interface, invargs, strlen(invargs));
		return 0;
	}

	/* Get operation argument location and size, store for later use */
	char *operation_arg = next_arg;
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
			const char *invargs = "GPIO pin values is ";
			parser_actions.send_resp(interface, invargs, strlen(invargs));

			if ((values & (1 << pin)) != 0) {
				parser_actions.send_resp(interface, "1", strlen("1"));
			} else {
				parser_actions.send_resp(interface, "0", strlen("0"));
			}

			parser_actions.send_resp(interface, "\r\n", strlen("\r\n"));
		}
	} else {
		const char *invargs = "Invalid operation for GPIO\r\n";
		parser_actions.send_resp(interface, invargs, strlen(invargs));
		return 0;
	}

	if (err == 0) {
		const char *okrsp = "GPIO ok\r\n";
		parser_actions.send_resp(interface, okrsp, strlen(okrsp));
	} else {
		const char *nokrsp = "GPIO failed\r\n";
		parser_actions.send_resp(interface, nokrsp, strlen(nokrsp));
	}

	return err;
}

/**
 * @brief Implementation of help command. 
 * 
 * @param[in] interface Interface that sent command. 
 * @param[in] arg String with arguments for command. 
 * 
 * @return 0 if ok, error code otherwise. 
 */
static int parser_help(enum diagnostics_interface interface, char *arg)
{
	const char *help = "Commands: help, gpio, test, modem, gnss\r\n";
	parser_actions.send_resp(interface, help, strlen(help));

	return 0;
}

/**
 * @brief Main command parser, searching for matching string. 
 * 
 * @param[in] interface Interface that sent command. 
 * @param[in] arg String with command. 
 * @param[in] length Length of command string. 
 * 
 * @return 0 if ok, error code otherwise. 
 */
static int parser_run(enum diagnostics_interface interface, char *cmd, uint32_t length)
{
	char *next_arg;
	uint32_t cmd_len = parser_get_next(cmd, &next_arg);
	if (cmd_len == 0) {
		const char *info = "-= Nofence collar diagnostics at your service =-\r\n"
				   "Type help for more information.\r\n";
		parser_actions.send_resp(interface, info, strlen(info));
		parser_actions.send_resp(interface, "Zephyr version is ",
					 strlen("Zephyr version is "));
		parser_actions.send_resp(interface, KERNEL_VERSION_STRING,
					 strlen(KERNEL_VERSION_STRING));
		parser_actions.send_resp(interface, "\r\n", strlen("\r\n"));
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
		parser_actions.send_resp(interface, "Unknown command\r\n",
					 strlen("Unknown command\r\n"));
	}

	return 0;
}

uint32_t parser_handle(enum diagnostics_interface interface, uint8_t *data, uint32_t size)
{
	uint32_t bytes_parsed = 0;

	uint32_t newline_loc = 0;
	if (parser_find_newline_offset(data, size, &newline_loc)) {
		bytes_parsed = newline_loc + 1;
		uint32_t cmd_length = newline_loc;
		while ((cmd_length >= 1) && (data[cmd_length - 1] == '\r')) {
			cmd_length--;
		}

		data[cmd_length] = '\0';
		parser_run(interface, data, cmd_length);
	}

	return bytes_parsed;
}