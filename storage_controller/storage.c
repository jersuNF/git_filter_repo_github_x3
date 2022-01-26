/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include <drivers/flash.h>
#include <device.h>
#include <devicetree.h>
#include <stdio.h>
#include <string.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(MODULE, CONFIG_STORAGE_CONTROLLER_LOG_LEVEL);

#define FLASH_DEVICE DT_LABEL(DT_INST(0, nordic_qspi_nor))
#define FLASH_NAME "JEDEC QSPI-NOR"

/* Thread stack area that we use for the storage controller operations 
 * such as read/write.
 */
K_THREAD_STACK_DEFINE(stg_thread_area, CONFIG_STORAGE_THREAD_SIZE);
static struct k_work_q stg_work_q;

/* Work items that are called from event handler. Currently stores
 * data so we do not hang/slow the event bus with flash write/read.
 */
struct read_container {
	struct k_work work;

	mem_rec *new_rec;
	flash_regions_t region;
};
struct write_container {
	struct k_work work;

	mem_rec *new_rec;
	flash_regions_t region;
};

/* Storage device. Used to identify the 
 * External Flash driver in the sensor API. 
 */
static const struct device *flash_dev;

void stg_controller_init(void)
{
	/* Init work item and start and init calculation 
	 * work queue thread and item. 
	 */
	k_work_queue_init(&stg_work_q);
	k_work_queue_start(&stg_work_q, stg_thread_area,
			   K_THREAD_STACK_SIZEOF(stg_thread_area),
			   CONFIG_STORAGE_THREAD_PRIORITY, NULL);
	k_work_init(&calc_work, calculate_work_fn);
}

/* Regions of where the different data is stored on flash.
 * Callers use enum flash_region_t to select LOG or ANO data region in events.
 */
static const flash_region flash_regions[] = {
	{ /** Circular buffer. */
	  .StartPtr = MEM_START_LOG,
	  .Length = MEM_MAX_SIZE_LOG,
	  .eepromWriteStart = &EEPROM_WriteRecSta4,
	  .eepromWriteEnd = &EEPROM_WriteRecEnd4,
	  .eepromWriteTransferred = &EEPROM_WriteRecTransferred4,
	  .eepromReadStart = &EEPROM_GetRecSta4,
	  .eepromReadEnd = &EEPROM_GetRecEnd4,
	  .eepromReadTransferred = &EEPROM_GetRecTransfered4 },

	{ /** Circular buffer. */
	  .StartPtr = MEM_START_ANO,
	  .Length = MEM_MAX_SIZE_ANO,
	  .eepromWriteStart = &EEPROM_WriteAnoStart4,
	  .eepromWriteEnd = &EEPROM_WriteAnoEnd4,
	  .eepromWriteTransferred = &EEPROM_WriteAnoTransferred4,
	  .eepromReadStart = &EEPROM_GetAnoStart4,
	  .eepromReadEnd = &EEPROM_GetAnoEnd4,
	  .eepromReadTransferred = &EEPROM_GetAnoTransferred4 }
};

int initialize_flash_driver(void)
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
 * @brief Main event handler function. 
 * 
 * @param[in] eh Event_header for the if-chain to 
 *               use to recognize which event triggered.
 * 
 * @return True or false based on if we want to consume the event or not.
 */

static bool event_handler(const struct event_header *eh)
{
	if (is_stg_read_memrec_event(eh)) {
		struct stg_read_memrec_event *ev =
			cast_stg_read_memrec_event(eh);

		return false;
	}
	if (is_stg_write_memrec_event(eh)) {
		struct stg_write_memrec_event *ev =
			cast_stg_write_memrec_event(eh);

		return false;
	}
	if (is_stg_ack_read_memrec_event(eh)) {
		/* Increment pointer to next area. */
		return false;
	}

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(MODULE, event_handler);

EVENT_SUBSCRIBE(MODULE, stg_write_memrec_event);
EVENT_SUBSCRIBE(MODULE, stg_read_memrec_event);
EVENT_SUBSCRIBE(MODULE, stg_ack_read_memrec_event);