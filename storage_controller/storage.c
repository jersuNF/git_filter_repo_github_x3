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

#include "ano_structure.h"
#include "log_structure.h"
#include "pasture_structure.h"
#include "storage.h"

#include "error_event.h"

#include "embedded.pb.h"

/* Get sizes and offset definitions from pm_static.yml. */
#include <pm_config.h>

#include <logging/log.h>
#define MODULE storage_controller
LOG_MODULE_REGISTER(MODULE, CONFIG_STORAGE_CONTROLLER_LOG_LEVEL);

const struct flash_area *log_area;
struct fcb log_fcb;
struct flash_sector log_sectors[FLASH_LOG_NUM_SECTORS];

const struct flash_area *ano_a_area;
struct fcb ano_a_fcb;
struct flash_sector ano_a_sectors[FLASH_ANO_A_NUM_SECTORS];

const struct flash_area *ano_b_area;
struct fcb ano_b_fcb;
struct flash_sector ano_b_sectors[FLASH_ANO_B_NUM_SECTORS];

static flash_partition_t ano_read_partition = STG_PARTITION_ANO_A;

const struct flash_area *pasture_area;
struct fcb pasture_fcb;
struct flash_sector pasture_sectors[FLASH_PASTURE_NUM_SECTORS];

K_MUTEX_DEFINE(log_mutex);
K_MUTEX_DEFINE(ano_a_mutex);
K_MUTEX_DEFINE(ano_b_mutex);
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
	} else if (partition == STG_PARTITION_ANO_A) {
		fcb = &ano_a_fcb;
	} else if (partition == STG_PARTITION_ANO_B) {
		fcb = &ano_b_fcb;
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
	} else if (partition == STG_PARTITION_ANO_A) {
		return &ano_a_mutex;
	} else if (partition == STG_PARTITION_ANO_B) {
		return &ano_b_mutex;
	} else if (partition == STG_PARTITION_PASTURE) {
		return &pasture_mutex;
	}
	LOG_ERR("Invalid partition given.");
	return NULL;
}

static inline flash_partition_t get_ano_read_partition()
{
	/* We default to partition A, but if B has data and not A, use B. */
	/* First checks if any of the partitions have any data. */
	bool is_a_empty = fcb_is_empty(get_fcb(STG_PARTITION_ANO_A));
	bool is_b_empty = fcb_is_empty(get_fcb(STG_PARTITION_ANO_B));

	if (is_a_empty && !is_b_empty) {
		return STG_PARTITION_ANO_B;
	}

	if (is_b_empty) {
		return STG_PARTITION_ANO_A;
	}

	/* If we get here, both of the partitions have data, we need to
	 * read out one entry to compare the version numbers.
	 */
	if (k_mutex_lock(&ano_a_mutex,
			 K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		LOG_ERR("Mutex A timeout in storage controller.");
		return -ETIMEDOUT;
	}

	/* If we get here, both of the partitions have data, we need to
	 * read out one entry to compare the version numbers.
	 */
	if (k_mutex_lock(&ano_b_mutex,
			 K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		LOG_ERR("Mutex B timeout in storage controller.");
		k_mutex_unlock(&ano_a_mutex);
		return -ETIMEDOUT;
	}

	UbxAnoReply ano_a;
	struct fcb *fcb_a = get_fcb(STG_PARTITION_ANO_A);
	int err = fcb_flash_read(fcb_a, 0, 0, (void *)&ano_a,
				 sizeof(UbxAnoReply));

	if (err) {
		LOG_ERR("Error reading from ANO partition A %i", err);
		k_mutex_unlock(&ano_a_mutex);
		k_mutex_unlock(&ano_b_mutex);
		return err;
	}

	UbxAnoReply ano_b;
	struct fcb *fcb_b = get_fcb(STG_PARTITION_ANO_B);
	err = fcb_flash_read(fcb_b, 0, 0, (void *)&ano_b, sizeof(UbxAnoReply));

	if (err) {
		LOG_ERR("Error reading from ANO partition B %i", err);
		k_mutex_unlock(&ano_a_mutex);
		k_mutex_unlock(&ano_b_mutex);
		return err;
	}
	k_mutex_unlock(&ano_a_mutex);
	k_mutex_unlock(&ano_b_mutex);
	return ano_b.usAnoId > ano_a.usAnoId ? STG_PARTITION_ANO_B :
					       STG_PARTITION_ANO_A;
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
	} else if (partition == STG_PARTITION_ANO_A) {
		sector_cnt = FLASH_ANO_A_NUM_SECTORS;
		area_id = FLASH_AREA_ID(ano_partition_a);
		area = ano_a_area;
		sector_ptr = ano_a_sectors;
	} else if (partition == STG_PARTITION_ANO_B) {
		sector_cnt = FLASH_ANO_B_NUM_SECTORS;
		area_id = FLASH_AREA_ID(ano_partition_a);
		area = ano_b_area;
		sector_ptr = ano_b_sectors;
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

	err = init_fcb_on_partition(STG_PARTITION_ANO_A);
	if (err) {
		return err;
	}

	err = init_fcb_on_partition(STG_PARTITION_ANO_B);
	if (err) {
		return err;
	}

	err = init_fcb_on_partition(STG_PARTITION_PASTURE);
	if (err) {
		return err;
	}

	/* Set active ANO partition based on version on storage controller. */
	ano_read_partition = get_ano_read_partition();
	LOG_INF("Active ano partition fetched was %i", ano_read_partition);

	return 0;
}

/** 
 * @brief Writes a frame to target partition. Needs to be used if we have 
 *        greater data contents than what we can buffer, so we have to split
 *        up the writing.
 * 
 * @param[in] data pointer location to of data to be written
 * @param[in] len length of data
 * @param[in] first_frame true if we're writing the first frame which
 *                        erases all previous entries.
 * 
 * @return 0 on success, otherwise negative errno
 */
int stg_write_frame_to_partition(flash_partition_t partition, uint8_t *data,
				 size_t len, bool first_frame)
{
	int err;

	/* Check if started a new entry or is on the same entry. */
	if (first_frame) {
		/* New entry, pass the TRUE flag to erase previous entries. */
		err = stg_write_to_partition(partition, data, len, true);
	} else {
		/* Same entry, pass the false flag to keep previous frames. */
		err = stg_write_to_partition(partition, data, len, false);
	}

	return err;
}

int stg_write_to_partition(flash_partition_t partition, uint8_t *data,
			   size_t len, bool rotate_to_this)
{
	struct fcb_entry loc;
	struct fcb *fcb = get_fcb(partition);
	int err = 0;

	/* If we only want to read the newest, clear all previous entries. */
	if (rotate_to_this) {
		err = fcb_clear(fcb);
		if (err) {
			return err;
		}
	}

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

	struct fcb *fcb = get_fcb(partition);
	int err = fcb_clear(fcb);

	k_mutex_unlock(mtx);
	return err;
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
				     stg_read_log_cb cb, bool rotate_after_read)
{
	int err = 0;

	/* Check if we have any data available. */
	struct fcb *fcb = get_fcb(partition);
	if (fcb_is_empty(fcb)) {
		err = -ENODATA;
		return err;
	}

	/* Length location used to store the entry location used
	 * filled in later by the storage controller and then given in
	 * the callback to the caller.
	 */
	size_t entry_len;

	/* Construct the config that is used as argument 
	 * to the callback function, this config contains all information
	 * the read operation requires. 
	 */
	struct walk_callback_config config;
	config.p = partition;
	config.len = &entry_len;

	if (cb == NULL) {
		err = -EINVAL;
		return err;
	}
	config.cb = cb;

	err = fcb_walk(fcb, NULL, walk_cb, &config);
	if (err) {
		LOG_ERR("Error walking over FCB storage, err %d", err);
		return err;
	}

	if (rotate_after_read) {
		err = fcb_clear(fcb);
		if (err) {
			LOG_ERR("Error clearing FCB after walk, err %d", err);
		}
	}
	return err;
}

int stg_read_log_data(stg_read_log_cb cb)
{
	if (k_mutex_lock(&log_mutex, K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		return -ETIMEDOUT;
	}
	int err = read_fcb_partition(STG_PARTITION_LOG, cb, true);
	if (err) {
		LOG_ERR("Error reading from log partition.");
	}
	k_mutex_unlock(&log_mutex);
	return err;
}

int stg_read_ano_data(stg_read_log_cb cb)
{
	struct k_mutex *mtx = get_mutex(ano_read_partition);
	if (k_mutex_lock(mtx, K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		return -ETIMEDOUT;
	}
	int err = read_fcb_partition(ano_read_partition, cb, false);
	if (err) {
		LOG_ERR("Error reading from ano partition.");
	}
	k_mutex_unlock(mtx);
	return err;
}

int stg_read_pasture_data(stg_read_log_cb cb)
{
	if (k_mutex_lock(&pasture_mutex,
			 K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		return -ETIMEDOUT;
	}
	int err = read_fcb_partition(STG_PARTITION_PASTURE, cb, false);
	if (err) {
		LOG_ERR("Error reading from pasture partition.");
	}
	k_mutex_unlock(&pasture_mutex);
	return err;
}

int stg_write_log_data(uint8_t *data, size_t len)
{
	if (k_mutex_lock(&log_mutex, K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		return -ETIMEDOUT;
	}
	int err = stg_write_to_partition(STG_PARTITION_LOG, data, len, false);
	if (err) {
		LOG_ERR("Error writing to log partition.");
	}
	k_mutex_unlock(&log_mutex);
	return err;
}

int stg_write_ano_data(uint8_t *data, size_t len, ano_frame_type frame_type)
{
	/* Check which partition we want to write to based on which is
	 * being used already.
	 */
	flash_partition_t ano_write_partition = STG_PARTITION_ANO_A;
	if (ano_read_partition == ano_write_partition) {
		ano_write_partition = STG_PARTITION_ANO_B;
	}

	struct k_mutex *mtx = get_mutex(ano_write_partition);
	if (k_mutex_lock(mtx, K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		return -ETIMEDOUT;
	}

	/* If its the first frame, we have to delete all the other entries (frames)
	 * that exists, since we can only have 1 ANO data on the
	 * partition.
	 */
	bool first_frame = frame_type == ANO_FRAME_FIRST;
	int err = stg_write_frame_to_partition(ano_write_partition, data, len,
					       first_frame);

	if (err) {
		LOG_ERR("Error writing to ano partition num %i, err %i",
			ano_write_partition, err);
		k_mutex_unlock(mtx);
		return err;
	}

	/* Default to A, but set to B if we're currently using A. Check what's
	 * currently active and choose the other. Only necessary to
	 * do on the last frame when everything is written to flash
	 * to ensure its valid.
	 */
	if (frame_type == ANO_FRAME_LAST) {
		ano_read_partition = STG_PARTITION_ANO_A;
		if (ano_write_partition == ano_read_partition) {
			ano_read_partition = STG_PARTITION_ANO_B;
		}
	}

	k_mutex_unlock(mtx);
	return err;
}

int stg_write_pasture_data(uint8_t *data, size_t len)
{
	if (k_mutex_lock(&pasture_mutex,
			 K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		return -ETIMEDOUT;
	}
	int err =
		stg_write_to_partition(STG_PARTITION_PASTURE, data, len, true);

	if (err) {
		LOG_ERR("Error writing to pasture partition %i", err);
	}
	k_mutex_unlock(&pasture_mutex);
	return err;
}

int stg_fcb_reset_and_init()
{
	memset(&log_fcb, 0, sizeof(log_fcb));
	memset(&ano_a_fcb, 0, sizeof(ano_a_fcb));
	memset(&ano_b_fcb, 0, sizeof(ano_b_fcb));
	memset(&pasture_fcb, 0, sizeof(pasture_fcb));

	return stg_init_storage_controller();
}