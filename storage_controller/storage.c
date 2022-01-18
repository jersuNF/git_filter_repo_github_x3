/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include <drivers/flash.h>
#include <device.h>
#include <devicetree.h>

#include "storage_event.h"
#include "config_event.h"
#include "modules_common.h"
#include <stdio.h>
#include <string.h>
#define MODULE storage_sim

#include <logging/log.h>
LOG_MODULE_REGISTER(MODULE, CONFIG_STORAGE_CONTROLLER_LOG_LEVEL);

#define SENSOR_SIMULATED_THREAD_STACK_SIZE 800
#define SENSOR_SIMULATED_THREAD_PRIORITY 1
#define SENSOR_SIMULATED_THREAD_SLEEP 500

#define FLASH_DEVICE DT_LABEL(DT_INST(0, nordic_qspi_nor))
#define FLASH_NAME "JEDEC QSPI-NOR"

/* Since erase is performed sector-wise we offset settings to write by sector size 4096 */
#define FLASH_DATA_SERIAL_NUMBER_REGION_OFFSET 0x000000
#define FLASH_DATA_FW_VERSION_REGION_OFFSET 0x001000
#define FLASH_DATA_IP_ADDRESS_REGION_OFFSET 0x002000

#define FLASH_SECTOR_SIZE 4096

bool initialized_module = false;

struct storage_msg_data {
	union {
		struct storage_event storage;
		struct config_event config;
	} module;
};

/* Storage module super states. */
static enum state_type { STATE_INIT, STATE_IDLE, STATE_ERROR } state;

/* Moduel data struct */
static struct module_data self = {
	.name = "storage",
	.msg_q = NULL,
	.supports_shutdown = true,
};

/* Storage device. Used to identify the External Flash driver in the sensor API. */
static const struct device *flash_dev;

/* Forward declarations. */
static void message_handler(struct storage_msg_data *data);

/**
 * @brief Useful functions to convert state to string.
 * @param new_state Enum with state type
 * @return char - string of state
 */
static char *state2str(enum state_type new_state)
{
	switch (new_state) {
	case STATE_INIT:
		return "STATE_INIT";
	case STATE_IDLE:
		return "STATE_IDLE";
	case STATE_ERROR:
		return "STATE_ERROR";
	default:
		return "Unknown";
	}
}

/**
 * @brief Set the state and log debug message 
 * @param new_state Enum containging the state we want to set
 */
static void state_set(enum state_type new_state)
{
	if (new_state == state) {
		LOG_DBG("State: %s", state2str(state));
		return;
	}

	LOG_DBG("State transition %s --> %s", state2str(state),
		state2str(new_state));

	state = new_state;
}

/**
 * @brief Setup external flash driver MX25R6435F on nRF52840dk
 * @param void
 * @return 0 if successful, otherwise a negative error code.
 */
static int setup_flash_driver(void)
{
	flash_dev = device_get_binding(FLASH_DEVICE);
	if (flash_dev == NULL) {
		LOG_ERR("Could not get  %s device!", log_strdup(FLASH_NAME));
		return -ENODEV;
	}
	LOG_DBG("Flash driver initialized");
	return 0;
}

/**
 * @brief Message handler for all states. 
 * @param msg Pointer to a structure containing the storage message data
 */
static void on_all_states(struct storage_msg_data *msg)
{
	if (IS_EVENT(msg, config, CONFIG_EVT_START)) {
		int err;

		err = module_start(&self);
		if (err) {
			LOG_ERR("Failed starting module, error: %d", err);
			SEND_ERROR(storage, STORAGE_EVT_ERROR_CODE, err);
		}

		/* Setup flash driver */
		err = setup_flash_driver();
		if (err) {
			LOG_ERR("setup flash driver error: %d", err);
			SEND_ERROR(storage, STORAGE_EVT_ERROR_CODE, err);
		}
	}
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
	/* Receive a storage event */
	if (is_storage_event(eh)) {
		struct storage_event *event = cast_storage_event(eh);
		struct storage_msg_data msg = { .module.storage = *event };
		message_handler(&msg);
	}
	/* Receievd a configuration event */
	if (is_config_event(eh)) {
		struct config_event *event = cast_config_event(eh);
		struct storage_msg_data msg = { .module.config = *event };

		message_handler(&msg);
	}

	return false;
}

/**
 * @brief Message handler for the initialization stage 
 * @param msg Pointer to a structure containing the storage message data
 */
static void on_state_init(struct storage_msg_data *msg)
{
	/* This boolean shuld be passed from a separate event, ref asset tracker */
	if (!initialized_module) {
		// Init flash module
		state_set(STATE_IDLE);
	}
	initialized_module = true;
}

/**
 * @brief Message handler for the idle stage 
 * @param msg Pointer to a structure containing the storage message data
 */
static void on_state_idle(struct storage_msg_data *msg)
{
	int err;

	/* Write serial number to flash */
	if (IS_EVENT(msg, storage, STORAGE_EVT_WRITE_SERIAL_NR)) {
		LOG_DBG("Erase flash sector");
		err = flash_erase(flash_dev,
				  FLASH_DATA_SERIAL_NUMBER_REGION_OFFSET,
				  FLASH_SECTOR_SIZE);
		if (err != 0) {
			LOG_ERR("Flash erase failed! %d", err);
			SEND_ERROR(storage, STORAGE_EVT_ERROR_CODE, err);
		}

		uint32_t data = msg->module.storage.data.pvt.serial_number;
		LOG_DBG("Write serial number %d to flash", data);
		err = flash_write(flash_dev,
				  FLASH_DATA_SERIAL_NUMBER_REGION_OFFSET, &data,
				  sizeof(data));
		if (err != 0) {
			LOG_ERR("Flash write failed! %d", err);
			SEND_ERROR(storage, STORAGE_EVT_ERROR_CODE, err);
		}
	}

	/* Read serial number from flash */
	if (IS_EVENT(msg, storage, STORAGE_EVT_READ_SERIAL_NR)) {
		uint32_t serial_number;
		err = flash_read(flash_dev,
				 FLASH_DATA_SERIAL_NUMBER_REGION_OFFSET,
				 &serial_number, sizeof(serial_number));
		if (err != 0) {
			LOG_ERR("Flash read failed! %d", err);
			SEND_ERROR(storage, STORAGE_EVT_ERROR_CODE, err);
		}
		LOG_DBG("Read serial number from flash: %d", serial_number);
		memcpy(&msg->module.storage.data.pvt.serial_number,
		       &serial_number, sizeof(serial_number));
	}

	/* Write fw version to flash */
	if (IS_EVENT(msg, storage, STORAGE_EVT_WRITE_FW_VERSION)) {
		LOG_DBG("Erase flash sector");
		err = flash_erase(flash_dev,
				  FLASH_DATA_FW_VERSION_REGION_OFFSET,
				  FLASH_SECTOR_SIZE);
		if (err != 0) {
			LOG_ERR("Flash erase failed! %d", err);
			SEND_ERROR(storage, STORAGE_EVT_ERROR_CODE, err);
		}
		LOG_DBG("Write fw_version %d-%d-%d to flash",
			msg->module.storage.data.pvt.firmware_version[0],
			msg->module.storage.data.pvt.firmware_version[1],
			msg->module.storage.data.pvt.firmware_version[2]);
		err = flash_write(
			flash_dev, FLASH_DATA_FW_VERSION_REGION_OFFSET,
			msg->module.storage.data.pvt.firmware_version,
			sizeof(msg->module.storage.data.pvt.firmware_version));
		if (err != 0) {
			LOG_ERR("Flash write failed! %d", err);
			SEND_ERROR(storage, STORAGE_EVT_ERROR_CODE, err);
		}
	}

	/* Read fw version from flash */
	if (IS_EVENT(msg, storage, STORAGE_EVT_READ_FW_VERSION)) {
		uint8_t fw_version[3];
		err = flash_read(flash_dev, FLASH_DATA_FW_VERSION_REGION_OFFSET,
				 fw_version, sizeof(fw_version));
		if (err != 0) {
			LOG_ERR("Flash read failed! %d", err);
			SEND_ERROR(storage, STORAGE_EVT_ERROR_CODE, err);
		}
		LOG_DBG("Read fw_version from flash: %d-%d-%d", fw_version[0],
			fw_version[1], fw_version[2]);
		memcpy(msg->module.storage.data.pvt.firmware_version,
		       fw_version, sizeof(fw_version));
	}

	/* Write IP-address to flash */
	if (IS_EVENT(msg, storage, STORAGE_EVT_WRITE_IP_ADDRESS)) {
		LOG_DBG("Erase flash sector");
		err = flash_erase(flash_dev,
				  FLASH_DATA_IP_ADDRESS_REGION_OFFSET,
				  FLASH_SECTOR_SIZE);
		if (err != 0) {
			LOG_ERR("Flash erase failed! %d", err);
			SEND_ERROR(storage, STORAGE_EVT_ERROR_CODE, err);
		}
		LOG_DBG("Write ip-address %d-%d-%d-%d to flash",
			msg->module.storage.data.pvt.server_ip_address[0],
			msg->module.storage.data.pvt.server_ip_address[1],
			msg->module.storage.data.pvt.server_ip_address[2],
			msg->module.storage.data.pvt.server_ip_address[3]);
		err = flash_write(
			flash_dev, FLASH_DATA_IP_ADDRESS_REGION_OFFSET,
			msg->module.storage.data.pvt.server_ip_address,
			sizeof(msg->module.storage.data.pvt.server_ip_address));
		if (err != 0) {
			LOG_ERR("Flash write failed! %d", err);
			SEND_ERROR(storage, STORAGE_EVT_ERROR_CODE, err);
		}
	}

	/* Read IP-address from flash */
	if (IS_EVENT(msg, storage, STORAGE_EVT_READ_IP_ADDRESS)) {
		uint8_t ip_address[4];
		err = flash_read(flash_dev, FLASH_DATA_IP_ADDRESS_REGION_OFFSET,
				 ip_address, sizeof(ip_address));
		if (err != 0) {
			LOG_ERR("Flash read failed! %d", err);
			SEND_ERROR(storage, STORAGE_EVT_ERROR_CODE, err);
		}
		LOG_DBG("Read ip-adderess from flash: [%d.%d.%d.%d]",
			ip_address[0], ip_address[1], ip_address[2],
			ip_address[3]);
		memcpy(msg->module.storage.data.pvt.server_ip_address,
		       ip_address, sizeof(ip_address));
	}

	/* Erase entire 64MB external flash, this might take a while... */
	if (IS_EVENT(msg, storage, STORAGE_EVT_ERASE_FLASH)) {
		LOG_DBG("Erase external flash");
		err = flash_erase(flash_dev, 0x00000, FLASH_SECTOR_SIZE * 2046);
		if (err != 0) {
			LOG_ERR("Flash erase failed! %d", err);
			SEND_ERROR(storage, STORAGE_EVT_ERROR_CODE, err);
		}
	}
}
/**
 * @brief Message handler function called by the event handler
 * @param msg Pointer to a structure containing the storage message data
 */
static void message_handler(struct storage_msg_data *msg)
{
	switch (state) {
	case STATE_INIT:
		on_state_init(msg);
		break;
	case STATE_IDLE:
		on_state_idle(msg);
		break;
	case STATE_ERROR:
		/* The error state has no transition. */
		break;
	default:
		LOG_ERR("Unknown storage module state.");
		break;
	}

	on_all_states(msg);
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, config_event);
EVENT_SUBSCRIBE(MODULE, storage_event);
