/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include <drivers/flash.h>
#include <storage/flash_map.h>
#include <device.h>
#include <devicetree.h>
#include <stdio.h>
#include <string.h>
#include "flash_memory.h"
#include "storage_events.h"
#include <fs/fcb.h>

/* Get sizes and offset definitions from pm_static.yml. */
#include <pm_config.h>

#include <logging/log.h>
#define MODULE storage_controller
LOG_MODULE_REGISTER(MODULE, CONFIG_STORAGE_CONTROLLER_LOG_LEVEL);

#define FLASH_LOG_NUM_SECTORS PM_LOG_PARTITION_SIZE / CONFIG_STORAGE_SECTOR_SIZE
#define FLASH_ANO_NUM_SECTORS PM_ANO_PARTITION_SIZE / CONFIG_STORAGE_SECTOR_SIZE

struct flash_area log_area;
struct fcb log_fcb;
struct flash_sector log_sectors[FLASH_LOG_NUM_SECTORS];

struct flash_area ano_area;
struct fcb ano_fcb;
struct flash_sector ano_sectors[FLASH_ANO_NUM_SECTORS];

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

int init_log_data_partition(void)
{
	int err;
	err = flash_area_has_driver(&log_area);
	if (err) {
		LOG_ERR("No device available for log area.");
		return err;
	}
	const struct flash_area *area = &log_area;
	err = flash_area_open(FLASH_AREA_ID(log_partition), &area);
	if (err) {
		LOG_ERR("Error opening flash area for log data.");
		return err;
	}

	/* Used to get flash parameters and driver specific attributes. */
	const struct device *dev;
	const struct flash_parameters *fp;

	dev = device_get_binding(log_area.fa_dev_name);
	flash_area_close(&log_area);

	if (dev == NULL) {
		LOG_ERR("Could not get device for log area!");
		return -ENODEV;
	}
	fp = flash_get_parameters(dev);

	uint32_t sector_cnt = FLASH_LOG_NUM_SECTORS;
	log_fcb.f_magic = 0U;
	log_fcb.f_erase_value = fp->erase_value;
	err = flash_area_get_sectors(FLASH_AREA_ID(log_partition), &sector_cnt,
				     log_sectors);
	if (err) {
		LOG_ERR("Unable to setup log sectors %d", err);
		return err;
	}
	if (sector_cnt > UINT8_MAX) {
		LOG_ERR("Number of log sectors got out of FCB range of 255.");
		return -ENOMEM;
	}
	log_fcb.f_sector_cnt = (uint8_t)sector_cnt;

	err = fcb_init(FLASH_AREA_ID(log_partition), &log_fcb);
	if (err) {
		LOG_ERR("Unable to initialize log flash circular buffer %d",
			err);
	}
	return err;
}

int init_ano_data_partition(void)
{
	int err;
	err = flash_area_has_driver(&ano_area);
	if (err) {
		LOG_ERR("No device available for ano area.");
		return err;
	}
	const struct flash_area *area = &ano_area;
	err = flash_area_open(FLASH_AREA_ID(ano_partition), &area);
	if (err) {
		LOG_ERR("Error opening flash area for ano data.");
		return err;
	}

	/* Used to get flash parameters and driver specific attributes. */
	const struct device *dev;
	const struct flash_parameters *fp;

	dev = device_get_binding(ano_area.fa_dev_name);
	flash_area_close(&ano_area);

	if (dev == NULL) {
		LOG_ERR("Could not get device for ano area!");
		return -ENODEV;
	}
	fp = flash_get_parameters(dev);

	uint32_t sector_cnt = FLASH_ANO_NUM_SECTORS;
	ano_fcb.f_magic = 0U;
	ano_fcb.f_erase_value = fp->erase_value;
	err = flash_area_get_sectors(FLASH_AREA_ID(ano_partition), &sector_cnt,
				     ano_sectors);
	if (err) {
		LOG_ERR("Unable to setup ano sectors %d", err);
		return err;
	}
	if (sector_cnt > UINT8_MAX) {
		LOG_ERR("Number of ano sectors got out of FCB range of 255.");
		return -ENOMEM;
	}
	ano_fcb.f_sector_cnt = (uint8_t)sector_cnt;

	err = fcb_init(FLASH_AREA_ID(ano_partition), &ano_fcb);
	if (err) {
		LOG_ERR("Unable to initialize ano flash circular buffer %d",
			err);
	}
	return err;
}

int init_storage_controller(void)
{
	int err;
	/* Init work item and start and init calculation 
	 * work queue thread and item. 
	 */
	k_work_queue_init(&stg_work_q);
	k_work_queue_start(&stg_work_q, stg_thread_area,
			   K_THREAD_STACK_SIZEOF(stg_thread_area),
			   CONFIG_STORAGE_THREAD_PRIORITY, NULL);

	err = init_ano_data_partition();
	if (err) {
		return err;
	}
	err = init_log_data_partition();
	if (err) {
		return err;
	}
	return 0;
}

void fcb_write_entry(flash_regions_t region, uint8_t *data, size_t len)
{
	/*int rc;
	struct fcb *fcb;
	struct fcb_entry loc;

	if (region == FLASH_REGION_LOG) {
		fcb = &log_fcb;
	} else if () {
		fcb = &ano_fcb;
	}

	for (i = 0; i < sizeof(test_data); i++) {
		rc = fcb_append(fcb, i, &loc);
		if (rc == -ENOSPC) {
			rc = fcb_rotate();
		}
		zassert_true(rc == 0, "fcb_append call failure");
		rc = flash_area_write(fcb->fap, FCB_ENTRY_FA_DATA_OFF(loc),
				      test_data, i);
		zassert_true(rc == 0, "flash_area_write call failure");
		rc = fcb_append_finish(fcb, &loc);
	}*/
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
		return false;
	}
	if (is_stg_write_memrec_event(eh)) {
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