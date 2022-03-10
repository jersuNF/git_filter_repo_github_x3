/*
 * Copyright (c) 2022 Nofence AS
 */

#include <zephyr.h>
#include <drivers/flash.h>
#include <storage/flash_map.h>
#include <device.h>
#include <devicetree.h>
#include <stdio.h>
#include <string.h>
#include <fs/fcb.h>

#include "fcb_ext.h"

#include "ano_structure.h"
#include "log_structure.h"
#include "pasture_structure.h"
#include "storage.h"
#include "storage_event.h"

#include "error_event.h"

#include "messaging_module_events.h"

#include "embedded.pb.h"
#include "UBX.h"

/* Get sizes and offset definitions from pm_static.yml. */
#include <pm_config.h>

#include <logging/log.h>
#define MODULE storage_controller
LOG_MODULE_REGISTER(MODULE, CONFIG_STORAGE_CONTROLLER_LOG_LEVEL);

const struct flash_area *log_area;
struct fcb log_fcb;
struct flash_sector log_sectors[FLASH_LOG_NUM_SECTORS];

static inline void update_ano_active_entry();
const struct flash_area *ano_area;
struct fcb ano_fcb;
struct flash_sector ano_sectors[FLASH_ANO_NUM_SECTORS];

struct fcb_entry active_ano_entry = { .fe_sector = NULL, .fe_elem_off = 0 };
struct fcb_entry last_sent_ano_entry = { .fe_sector = NULL, .fe_elem_off = 0 };

struct fcb_entry active_log_entry = { .fe_sector = NULL, .fe_elem_off = 0 };

const struct flash_area *pasture_area;
struct fcb pasture_fcb;
struct flash_sector pasture_sectors[FLASH_PASTURE_NUM_SECTORS];

K_MUTEX_DEFINE(log_mutex);
K_MUTEX_DEFINE(ano_mutex);
K_MUTEX_DEFINE(pasture_mutex);

K_KERNEL_STACK_DEFINE(erase_flash_thread, CONFIG_STORAGE_THREAD_SIZE);

void erase_flash_fn(struct k_work *item);
struct k_work_q erase_q;
struct k_work erase_work;
static bool queue_inited = false;

void erase_flash_fn(struct k_work *item)
{
	int err = stg_clear_partition(STG_PARTITION_LOG);
	if (err) {
		return;
	}
	err = stg_clear_partition(STG_PARTITION_ANO);
	if (err) {
		return;
	}
	err = stg_clear_partition(STG_PARTITION_PASTURE);
	if (err) {
		return;
	}

	struct update_flash_erase *ev = new_update_flash_erase();
	EVENT_SUBMIT(ev);
}

static volatile bool has_inited = false;

/** @brief Gets fcb structure based on partition.
 * 
 * @param partition which partition to get fcb from
 * 
 * @return pointer to fcb structure
 */
struct fcb *get_fcb(flash_partition_t partition)
{
	struct fcb *fcb;

	if (partition == STG_PARTITION_LOG) {
		fcb = &log_fcb;
	} else if (partition == STG_PARTITION_ANO) {
		fcb = &ano_fcb;
	} else if (partition == STG_PARTITION_PASTURE) {
		fcb = &pasture_fcb;
	} else {
		LOG_ERR("Invalid partition given.");
		return NULL;
	}
	return fcb;
}

/** @brief Gets mutex to lock/unlock based on partition.
 * 
 * @param partition which partition to get mutex from
 * 
 * @return pointer to mutex structure
 */
struct k_mutex *get_mutex(flash_partition_t partition)
{
	if (partition == STG_PARTITION_LOG) {
		return &log_mutex;
	} else if (partition == STG_PARTITION_ANO) {
		return &ano_mutex;
	} else if (partition == STG_PARTITION_PASTURE) {
		return &pasture_mutex;
	}
	LOG_ERR("Invalid partition given.");
	return NULL;
}

static inline int init_fcb_on_partition(flash_partition_t partition)
{
	int err;
	const struct device *dev;
	uint32_t sector_cnt;
	int area_id;
	const struct flash_area *area;
	struct flash_sector *sector_ptr;
	struct fcb *fcb = get_fcb(partition);

	/* Based on parameter, setup variables required by FCB. */
	if (partition == STG_PARTITION_LOG) {
		sector_cnt = FLASH_LOG_NUM_SECTORS;
		area_id = FLASH_AREA_ID(log_partition);
		area = log_area;
		sector_ptr = log_sectors;
	} else if (partition == STG_PARTITION_ANO) {
		sector_cnt = FLASH_ANO_NUM_SECTORS;
		area_id = FLASH_AREA_ID(ano_partition);
		area = ano_area;
		sector_ptr = ano_sectors;
	} else if (partition == STG_PARTITION_PASTURE) {
		sector_cnt = FLASH_PASTURE_NUM_SECTORS;
		area_id = FLASH_AREA_ID(pasture_partition);
		area = pasture_area;
		sector_ptr = pasture_sectors;
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

	has_inited = true;
	return err;
}

int stg_init_storage_controller(void)
{
	int err;

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

	err = init_fcb_on_partition(STG_PARTITION_PASTURE);
	if (err) {
		return err;
	}

	/* Setup work threads. */
	if (!queue_inited) {
		LOG_INF("Initializing messaging module!\n");
		k_work_queue_init(&erase_q);
		k_work_queue_start(&erase_q, erase_flash_thread,
				   K_THREAD_STACK_SIZEOF(erase_flash_thread),
				   CONFIG_STORAGE_THREAD_PRIORITY, NULL);
		k_work_init(&erase_work, erase_flash_fn);
		queue_inited = true;
	}

	/* Check which ANO partition is active. We just booted, so
	 * we have to go through every entry.
	 */
	update_ano_active_entry(NULL);

	return 0;
}

int stg_write_to_partition(flash_partition_t partition, uint8_t *data,
			   size_t len)
{
	struct fcb_entry loc;
	struct fcb *fcb = get_fcb(partition);
	int err = 0;

	/* Appending a new entry, rotate(replaces) oldest if no space. */
	err = fcb_append(fcb, len, &loc);
	if (err == -ENOSPC) {
		err = fcb_rotate(fcb);
		if (err) {
			LOG_ERR("Unable to rotate fcb from -ENOSPC, err %d",
				err);
			return err;
		}
		/* Retry appending. */
		err = fcb_append(fcb, len, &loc);
		if (err) {
			LOG_ERR("Unable to recover in appending function, err %d",
				err);
			nf_app_error(ERR_SENDER_STORAGE_CONTROLLER,
				     -ENOTRECOVERABLE, NULL, 0);
			return err;
		}
		LOG_INF("Rotated FCB since it's full.");
	} else if (err) {
		LOG_ERR("Error appending new fcb entry, err %d", err);
		return err;
	}

	err = flash_area_write(fcb->fap, FCB_ENTRY_FA_DATA_OFF(loc), data, len);
	if (err) {
		LOG_ERR("Error writing to flash area. err %d", err);
		return err;
	}

	/* Finish entry. */
	err = fcb_append_finish(fcb, &loc);
	if (err) {
		LOG_ERR("Error finishing new entry. err %d", err);
	}
	return err;
}

int stg_clear_partition(flash_partition_t partition)
{
	struct k_mutex *mtx = get_mutex(partition);

	if (mtx == NULL) {
		return -EINVAL;
	}

	if (k_mutex_lock(mtx, K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		LOG_ERR("Mutex timeout in storage controller when clearing.");
		return -ETIMEDOUT;
	}

	if (partition == STG_PARTITION_ANO) {
		active_ano_entry.fe_sector = NULL;
		active_ano_entry.fe_elem_off = 0;

		last_sent_ano_entry.fe_sector = NULL;
		last_sent_ano_entry.fe_elem_off = 0;
	} else if (partition == STG_PARTITION_LOG) {
		active_log_entry.fe_sector = NULL;
		active_log_entry.fe_elem_off = 0;
	}

	struct fcb *fcb = get_fcb(partition);
	int err = fcb_clear(fcb);

	k_mutex_unlock(mtx);
	return err;
}

int stg_read_log_data(fcb_read_cb cb, uint16_t num_entries)
{
	if (k_mutex_lock(&log_mutex, K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		return -ETIMEDOUT;
	}

	if (fcb_is_empty(&log_fcb)) {
		k_mutex_unlock(&ano_mutex);
		return -ENODATA;
	}

	struct fcb_entry start_entry;

	memcpy(&start_entry, &active_log_entry, sizeof(struct fcb_entry));

	int err = fcb_getnext(&log_fcb, &start_entry);
	if (err) {
		LOG_ERR("Error fetching next log entry %i", err);
		k_mutex_unlock(&ano_mutex);
		return err;
	}

	err = fcb_walk_from_entry(cb, &log_fcb, &start_entry, num_entries);
	if (err && err != -EINTR) {
		LOG_ERR("Error reading from log partition.");
	}

	/* Update the entry we're currently on. */
	memcpy(&active_log_entry, &start_entry, sizeof(struct fcb_entry));

	k_mutex_unlock(&log_mutex);
	return err;
}

/** @brief When condition is met that we have a valid ANO partition,
 *         return -EINTR.
 * @param[in] data data to check
 * @param[in] len size of data
 * 
 * @returns 0 if we want to search more ANO frames.
 * @returns -EINTR if we found a valid frame, stopping the walk process.
 */
int check_if_ano_valid_cb(uint8_t *data, size_t len)
{
	UBX_MGA_ANO_RAW_t *ano_frame = (UBX_MGA_ANO_RAW_t *)(data);

	/* Age logic here. */
	LOG_INF("Year %i, month %i, day %i", ano_frame->mga_ano.year,
		ano_frame->mga_ano.month, ano_frame->mga_ano.day);

	/* If age is valid, update ano sector. Fetch from somewhere! */
	if (ano_frame->mga_ano.year == 22) {
		/* Done, we found oldest, valid timestamp, exit loop. */
		return -EINTR;
	}
	return 0;
}

/** @brief Goes through ANO partition data and stops whenever
 *         it reaches a valid ano frame based on logic from function 
 *         check_if_ano_valid_cb. When traversing stops with -EINTR, we're done
 *         and active_ano_entry is kept on RAM for future reads.
 * 
 * @param entry Entry to start reading ANO data from. If NULL, traverse entire
 *              storage.
 */
static inline void update_ano_active_entry(struct fcb_entry *entry)
{
	if (k_mutex_lock(&ano_mutex, K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		return;
	}

	struct fcb_entry start_entry = { .fe_sector = NULL, .fe_elem_off = 0 };
	if (entry != NULL) {
		memcpy(&start_entry, entry, sizeof(struct fcb_entry));
	}

	int err = fcb_walk_from_entry(check_if_ano_valid_cb, &ano_fcb,
				      &start_entry, 0);

	if (err == -EINTR) {
		/* Found valid boot partition, copy the entry header. */
		memcpy(&active_ano_entry, &start_entry,
		       sizeof(struct fcb_entry));
	} else if (err == -ENODATA) {
		LOG_WRN("No ano frames available at ano partition.");
	} else if (err) {
		LOG_ERR("Error fetching ANO entries when updating %i", err);
	}

	k_mutex_unlock(&ano_mutex);
	return;
}

int stg_read_ano_data(fcb_read_cb cb, bool last_valid_ano, uint16_t num_entries)
{
	if (k_mutex_lock(&ano_mutex, K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		return -ETIMEDOUT;
	}

	int err = 0;

	if (fcb_is_empty(&ano_fcb)) {
		LOG_WRN("No ano frames found on partition.");
		k_mutex_unlock(&ano_mutex);
		return -ENODATA;
	}

	struct fcb_entry start_entry;

	if (last_valid_ano) {
		if (active_ano_entry.fe_sector == NULL) {
			k_mutex_unlock(&ano_mutex);
			return -EINVAL;
		}
		memcpy(&start_entry, &active_ano_entry,
		       sizeof(struct fcb_entry));
	} else {
		memcpy(&start_entry, &last_sent_ano_entry,
		       sizeof(struct fcb_entry));
	}

	err = fcb_getnext(&ano_fcb, &start_entry);
	if (err) {
		LOG_ERR("Error fetching next ano entry %i", err);
		k_mutex_unlock(&ano_mutex);
		return err;
	}

	err = fcb_walk_from_entry(cb, &ano_fcb, &start_entry, num_entries);

	if (err && err != -EINTR) {
		k_mutex_unlock(&ano_mutex);
		return err;
	}

	/* We know start_entry has been filled with the last read entry,
	 * so we can simply memcpy the contents.
	 */
	memcpy(&last_sent_ano_entry, &start_entry, sizeof(struct fcb_entry));

	k_mutex_unlock(&ano_mutex);
	return 0;
}

int stg_read_pasture_data(fcb_read_cb cb)
{
	if (k_mutex_lock(&pasture_mutex,
			 K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		return -ETIMEDOUT;
	}

	struct fcb *fcb = get_fcb(STG_PARTITION_PASTURE);

	if (fcb_is_empty(fcb)) {
		return -ENODATA;
	}

	struct fcb_entry entry;
	int err = fcb_offset_last_n(fcb, 1, &entry);

	if (err) {
		k_mutex_unlock(&pasture_mutex);
		return err;
	}

	size_t fence_size = entry.fe_data_len;
	uint8_t *fence = k_malloc(fence_size);

	err = flash_area_read(fcb->fap, FCB_ENTRY_FA_DATA_OFF(entry), fence,
			      fence_size);
	if (err) {
		k_free(fence);
		k_mutex_unlock(&pasture_mutex);
		return err;
	}

	cb(fence, fence_size);
	k_free(fence);

	k_mutex_unlock(&pasture_mutex);
	return err;
}

int stg_write_log_data(uint8_t *data, size_t len)
{
	if (k_mutex_lock(&log_mutex, K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		return -ETIMEDOUT;
	}

	int err = stg_write_to_partition(STG_PARTITION_LOG, data, len);

	if (err) {
		LOG_ERR("Error writing to log partition.");
	}

	k_mutex_unlock(&log_mutex);
	return err;
}

int stg_write_ano_data(uint8_t *data, size_t len)
{
	if (k_mutex_lock(&ano_mutex, K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		return -ETIMEDOUT;
	}

	int err = stg_write_to_partition(STG_PARTITION_ANO, data, len);

	if (err) {
		LOG_ERR("Error writing to ano partition, err %i", err);
	}

	k_mutex_unlock(&ano_mutex);
	return err;
}

int stg_write_pasture_data(uint8_t *data, size_t len)
{
	if (k_mutex_lock(&pasture_mutex,
			 K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		return -ETIMEDOUT;
	}

	int err = stg_write_to_partition(STG_PARTITION_PASTURE, data, len);

	if (err) {
		LOG_ERR("Error writing to pasture partition %i", err);
	}

	k_mutex_unlock(&pasture_mutex);
	return err;
}

int stg_fcb_reset_and_init()
{
	memset(&log_fcb, 0, sizeof(log_fcb));
	memset(&ano_fcb, 0, sizeof(ano_fcb));
	memset(&pasture_fcb, 0, sizeof(pasture_fcb));

	return stg_init_storage_controller();
}

static bool event_handler(const struct event_header *eh)
{
	if (is_request_flash_erase_event(eh)) {
		k_work_submit_to_queue(&erase_q, &erase_work);
		return true;
	}

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, request_flash_erase_event);