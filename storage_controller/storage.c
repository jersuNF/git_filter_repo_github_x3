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

const struct flash_area *log_area;
struct fcb log_fcb;
struct flash_sector log_sectors[FLASH_LOG_NUM_SECTORS];

const struct flash_area *ano_area;
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
	flash_partition_t region;
};
struct write_container {
	struct k_work work;

	mem_rec *new_rec;
	flash_partition_t region;
};

static inline int init_fcb_on_partition(flash_partition_t partition)
{
	int err;
	const struct device *dev;
	struct fcb *fcb;
	uint32_t sector_cnt;
	int area_id;
	const struct flash_area *area;
	struct flash_sector *sector_ptr;

	/* Based on parameter, setup variables required by FCB. */
	if (partition == STG_PARTITION_LOG) {
		fcb = &log_fcb;
		sector_cnt = FLASH_LOG_NUM_SECTORS;
		area_id = FLASH_AREA_ID(log_partition);
		area = log_area;
		sector_ptr = log_sectors;
	} else if (partition == STG_PARTITION_ANO) {
		fcb = &ano_fcb;
		sector_cnt = FLASH_ANO_NUM_SECTORS;
		area_id = FLASH_AREA_ID(ano_partition);
		area = ano_area;
		sector_ptr = ano_sectors;
	} else {
		LOG_ERR("Invalid partition given. %d", -EINVAL);
		return -EINVAL;
	}

	err = flash_area_open(area_id, &area);
	if (err) {
		LOG_ERR("Error opening flash area for partition %d, err %d",
			partition, err);
		return err;
	}

	/* Check if area has a flash device available. */
	dev = device_get_binding(area->fa_dev_name);
	flash_area_close(area);

	if (dev == NULL) {
		LOG_ERR("Could not get device for partition %d.", partition);
		return -ENODEV;
	}

	/* Used to get flash parameters and driver specific attributes. */
	const struct flash_parameters *fp;
	fp = flash_get_parameters(dev);

	/* Setup a fresh flash circular buffer, 
	 * this does not EMPTY existing contents on flash, but is
	 * required for fcb_init to work properly. 
	 */
	(void)memset(fcb, 0, sizeof(*fcb));

	fcb->f_magic = 0U;
	fcb->f_erase_value = fp->erase_value;

	/* Setup the sectors. We input sector_cnt to be MAX 255
	 * since this is the limitation of FCB. Sector count is
	 * IN,OUT parameter, in which case it retrieves the MAX
	 * number of sectors, and outputs how many is available
	 * based on partition size.
	 */
	err = flash_area_get_sectors(area_id, &sector_cnt, sector_ptr);
	if (err) {
		LOG_ERR("Unable to setup sectors for partition %d, err %d.",
			partition, err);
		return err;
	}

	fcb->f_sector_cnt = (uint8_t)sector_cnt;
	fcb->f_sectors = sector_ptr;

	err = fcb_init(area_id, fcb);
	if (err) {
		LOG_ERR("Unable to initialize fcb for partition %d, err %d.",
			partition, err);
		return err;
	}

	LOG_INF("Setup FCB for partition %d: %d sectors with sizes %db.",
		partition, fcb->f_sector_cnt, fcb->f_sectors[0].fs_size);
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

	/* Initialize FCB on LOG and ANO partitions
	 * based on pm_static.yml/.dts flash setup.
	 */
	err = init_fcb_on_partition(STG_PARTITION_LOG);
	if (err) {
		return err;
	}

	err = init_fcb_on_partition(STG_PARTITION_ANO);
	if (err) {
		return err;
	}
	return 0;
}

void fcb_write_entry(flash_partition_t region, uint8_t *data, size_t len)
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