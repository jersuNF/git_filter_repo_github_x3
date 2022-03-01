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
#include <fs/fcb.h>

#include "ano_structure.h"
#include "log_structure.h"
#include "pasture_structure.h"
#include "storage.h"
#include "pasture_event.h"

#include "error_event.h"

/* Get sizes and offset definitions from pm_static.yml. */
#include <pm_config.h>

#include <logging/log.h>
#define MODULE storage_controller
LOG_MODULE_REGISTER(MODULE, CONFIG_STORAGE_CONTROLLER_LOG_LEVEL);

const struct flash_area *log_area;
struct fcb log_fcb;
struct flash_sector log_sectors[FLASH_LOG_NUM_SECTORS];

const struct flash_area *ano_area;
struct fcb ano_fcb;
struct flash_sector ano_sectors[FLASH_ANO_NUM_SECTORS];

const struct flash_area *pasture_area;
struct fcb pasture_fcb;
struct flash_sector pasture_sectors[FLASH_PASTURE_NUM_SECTORS];

K_MUTEX_DEFINE(log_mutex);
K_MUTEX_DEFINE(ano_mutex);
K_MUTEX_DEFINE(pasture_mutex);

/* Callback context config given to walk callback function. 
 * We can either give the data directly if only one entry (newest),
 * or we can call the callback function for each entry read.
 * If data and len is NULL, cb should be given, and vice versa.
 */
struct walk_callback_config {
	uint8_t *data;
	size_t *len;
	stg_read_log_cb cb;
	flash_partition_t p;

	/* Not user configurable. Used by FCB. */
	off_t offset;
};

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

	return 0;
}

int stg_write_to_partition(flash_partition_t partition, uint8_t *data,
			   size_t len)
{
	struct k_mutex *mtx = get_mutex(partition);

	if (mtx == NULL) {
		return -EINVAL;
	}

	if (k_mutex_lock(mtx, K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		LOG_ERR("Mutex timeout in storage controller.");
		return -ETIMEDOUT;
	}
	/* Determine write type. If true, we only have this newest entry 
	 * to be written below on the flash partition. */
	bool rotate_to_this = true;

	if (partition == STG_PARTITION_LOG) {
		rotate_to_this = false;
	}

	struct fcb_entry loc;
	struct fcb *fcb = get_fcb(partition);
	int err = 0;

	/* If we only want to read the newest, clear all previous entries. */
	if (rotate_to_this) {
		err = fcb_clear(fcb);
		if (err) {
			goto cleanup;
		}
	}

	/* Appending a new entry, rotate(replaces) oldest if no space. */
	err = fcb_append(fcb, len, &loc);
	if (err == -ENOSPC) {
		err = fcb_rotate(fcb);
		if (err) {
			LOG_ERR("Unable to rotate fcb from -ENOSPC, err %d",
				err);
			goto cleanup;
		}
		/* Retry appending. */
		err = fcb_append(fcb, len, &loc);
		if (err) {
			LOG_ERR("Unable to recover in appending function, err %d",
				err);
			nf_app_error(ERR_SENDER_STORAGE_CONTROLLER,
				     -ENOTRECOVERABLE, NULL, 0);
			goto cleanup;
		}
		LOG_INF("Rotated FCB since it's full.");
	} else if (err) {
		LOG_ERR("Error appending new fcb entry, err %d", err);
		goto cleanup;
	}

	err = flash_area_write(fcb->fap, FCB_ENTRY_FA_DATA_OFF(loc), data, len);
	if (err) {
		LOG_ERR("Error writing to flash area. err %d", err);
		goto cleanup;
	}

	/* Finish entry. */
	err = fcb_append_finish(fcb, &loc);
	if (err) {
		LOG_ERR("Error finishing new entry. err %d", err);
		goto cleanup;
	}

	/* Publish pastrure ready event for those who need. */
	if (partition == STG_PARTITION_PASTURE) {
		struct pasture_ready_event *ev = new_pasture_ready_event();
		EVENT_SUBMIT(ev);
	}
cleanup:
	k_mutex_unlock(mtx);
	return err;
}

int stg_clear_partition(flash_partition_t partition)
{
	struct fcb *fcb = get_fcb(partition);
	return fcb_clear(fcb);
}

/** @brief Helper function for reading entry from external flash. Calls
 *         the necessary callback function if it's not NULL, stores it
 *         into the config DATA/LEN if no callback is given.
 * 
 * @param[in] config walk callback config containing all the information
 *                   the raw read function requires, such as offsets, lengths,
 *                   target callback, data pointer etc..
 * 
 * @return 0 on success, otherwise negative errno.
 */
static inline int stg_read_entry_raw(struct walk_callback_config *config)
{
	struct fcb *fcb = get_fcb(config->p);

	int err = 0;
	err = flash_area_read(fcb->fap, config->offset, config->data,
			      *config->len);

	if (err) {
		LOG_ERR("Error reading from flash to callback, err %d", err);
		return err;
	}

	/* Callback function with the contents to the caller. */
	err = config->cb(config->data, *config->len);

	if (err) {
		LOG_ERR("Error from user callback, err %d", err);
	}
	return err;
}

static int walk_cb(struct fcb_entry_ctx *loc_ctx, void *arg)
{
	struct walk_callback_config *config =
		(struct walk_callback_config *)arg;

	config->offset = FCB_ENTRY_FA_DATA_OFF(loc_ctx->loc);
	*config->len = loc_ctx->loc.fe_data_len;

	/* Allocate buffer for the data. */
	config->data = (uint8_t *)k_malloc(*config->len);
	if (config->data == NULL) {
		LOG_ERR("No memory left for allocation of entry.");
		return -ENOMEM;
	}

	int err = stg_read_entry_raw(config);
	k_free(config->data);
	return err;
}

static inline int read_fcb_partition(flash_partition_t partition,
				     stg_read_log_cb cb)
{
	int err = 0;

	struct k_mutex *mtx = get_mutex(partition);

	if (mtx == NULL) {
		return -EINVAL;
	}

	if (k_mutex_lock(mtx, K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		LOG_ERR("Mutex timeout in storage controller.");
		return -ETIMEDOUT;
	}

	/* Check if we have any data available. */
	struct fcb *fcb = get_fcb(partition);
	if (fcb_is_empty(fcb)) {
		err = -ENODATA;
		goto cleanup;
	}

	/* Length location used to store the entry location used
	 * filled in later by the storage controller and then given in
	 * the callback to the caller.
	 */
	size_t entry_len;

	/* Determine the read type. Either read the newest entry only,
	 * or read every entry not read previously.
	 */
	bool read_only_newest = true;

	if (partition == STG_PARTITION_LOG) {
		read_only_newest = false;
	}

	/* Construct the config that is used as argument 
	 * to the callback function, this config contains all information
	 * the read operation requires. 
	 */
	struct walk_callback_config config;
	config.p = partition;
	config.len = &entry_len;

	if (cb == NULL) {
		err = -EINVAL;
		goto cleanup;
	}
	config.cb = cb;

	err = fcb_walk(fcb, NULL, walk_cb, &config);
	if (err) {
		LOG_ERR("Error walking over FCB storage, err %d", err);
		goto cleanup;
	}

	if (!read_only_newest) {
		err = fcb_clear(fcb);
		if (err) {
			LOG_ERR("Error clearing FCB after walk, err %d", err);
			goto cleanup;
		}
	}
cleanup:
	k_mutex_unlock(mtx);
	return err;
}

int stg_read_log_data(stg_read_log_cb cb)
{
	return read_fcb_partition(STG_PARTITION_LOG, cb);
}

int stg_read_ano_data(stg_read_log_cb cb)
{
	return read_fcb_partition(STG_PARTITION_ANO, cb);
}

int stg_read_pasture_data(stg_read_log_cb cb)
{
	return read_fcb_partition(STG_PARTITION_PASTURE, cb);
}

int stg_fcb_reset_and_init()
{
	memset(&log_fcb, 0, sizeof(log_fcb));
	memset(&ano_fcb, 0, sizeof(ano_fcb));
	memset(&pasture_fcb, 0, sizeof(pasture_fcb));

	return stg_init_storage_controller();
}