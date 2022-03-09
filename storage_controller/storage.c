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

static inline void update_ano_active_index();
const struct flash_area *ano_area;
struct fcb ano_fcb;
struct flash_sector ano_sectors[FLASH_ANO_NUM_SECTORS];
struct flash_sector *active_ano_sector = &ano_sectors[0];

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

	/* Check which ANO partition is active. */
	update_ano_active_index();

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
	/*if (rotate_to_this) {
		err = fcb_clear(fcb);
		if (err) {
			return err;
		}
	}*/

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

/** @brief Reads fcb partition and callbacks all the entries found on the sector.
 *         If sector == NULL, then we traverse entire storage.
 * 
 * @param partition partition to read from.
 * @param sector sector to read from. If null, read entire partition.
 * @param cb callback that gives the entries found.
 * @param rotate_after_read rotates f_oldest() if true and a sector is given, 
 *                          clears partition if sector == NULL. Does nothing 
 *                          if false.
 * 
 * @return 0 on success, otherwise negative errno.
 */
static inline int read_fcb_partition(flash_partition_t partition,
				     struct flash_sector *sector,
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

	err = fcb_walk(fcb, sector, walk_cb, &config);
	if (err) {
		LOG_ERR("Error walking over FCB storage, err %d", err);
		return err;
	}

	if (rotate_after_read) {
		if (sector == NULL) {
			err = fcb_clear(fcb);
			if (err) {
				LOG_ERR("Error clearing FCB after walk, err %d",
					err);
			}
		} else {
			err = fcb_rotate(fcb);
		}
	}
	return err;
}

int stg_read_log_data(stg_read_log_cb cb)
{
	if (k_mutex_lock(&log_mutex, K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		return -ETIMEDOUT;
	}
	int err = read_fcb_partition(STG_PARTITION_LOG, NULL, cb, true);
	if (err) {
		LOG_ERR("Error reading from log partition.");
	}
	k_mutex_unlock(&log_mutex);
	return err;
}

static struct flash_sector *cur_sector;

int check_if_ano_valid_cb(uint8_t *data, size_t len)
{
	UBX_MGA_ANO_RAW_t *ano_frame = (UBX_MGA_ANO_RAW_t *)(data);

	/* Age logic here. */
	LOG_INF("Year %i, month %i, day %i", ano_frame->mga_ano.year,
		ano_frame->mga_ano.month, ano_frame->mga_ano.day);

	/* If age is valid, update ano sector. Fetch from somewhere! */
	if (ano_frame->mga_ano.year == 22) {
		LOG_INF("Updated offset to be %i", (int)cur_sector->fs_off);
		active_ano_sector = cur_sector;
		/* Done, we found oldest, valid timestamp, exit loop. */
		return -EINTR;
	}
	return 0;
}

/** @brief Helper function to increment a sector to the next.
 * 
 * @param[in] fcb fcb to target.
 * @param[inout] sector which sector to increment.
 */
static inline void increment_sector(struct fcb *fcb,
				    struct flash_sector *sector)
{
	sector++;
	if (sector >= &fcb->f_sectors[fcb->f_sector_cnt]) {
		sector = &fcb->f_sectors[0];
	}
}

static inline void update_ano_active_index()
{
	struct fcb *fcb = get_fcb(STG_PARTITION_ANO);

	cur_sector = fcb->f_oldest;

	int err = 0;
	int sectors_checked = 0;

	while (true) {
		err = read_fcb_partition(STG_PARTITION_ANO, cur_sector,
					 check_if_ano_valid_cb, false);

		if (err) {
			if (err == -EINTR) {
				LOG_INF("Updated ANO index partition to be offset %i",
					(int)active_ano_sector->fs_off);
				return;
			} else if (err == -ENODATA) {
				goto no_valid_ano;
			} else {
				LOG_ERR("Error reading ano to update index %i",
					err);
				return;
			}
		}

		increment_sector(fcb, cur_sector);

		sectors_checked++;

		if (sectors_checked > fcb->f_sector_cnt) {
			/* Exit if we reached the end, meaning
			 * no ANO data is valid... 
			 */
			goto no_valid_ano;
		}
	}
no_valid_ano:
	LOG_WRN("No valid ANO data on the partition. %i", err);
	return;
}

int stg_read_ano_data(stg_read_log_cb cb)
{
	if (k_mutex_lock(&ano_mutex, K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		return -ETIMEDOUT;
	}
	int err = read_fcb_partition(STG_PARTITION_ANO, NULL, cb, false);
	if (err) {
		LOG_ERR("Error reading from ano partition.");
	}
	k_mutex_unlock(&ano_mutex);
	return err;
}

int stg_read_pasture_data(stg_read_log_cb cb)
{
	if (k_mutex_lock(&pasture_mutex,
			 K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		return -ETIMEDOUT;
	}
	/*int err = read_fcb_partition(STG_PARTITION_PASTURE, NULL, cb, false);
	if (err) {
		LOG_ERR("Error reading from pasture partition.");
	}*/
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

int stg_write_ano_data(uint8_t *data, size_t len, bool first_frame)
{
	if (k_mutex_lock(&ano_mutex, K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		return -ETIMEDOUT;
	}
	/* If its the first frame, we have to delete all the other entries (frames)
	 * that exists, since we can only have 1 ANO data on the
	 * partition.
	 */
	int err = stg_write_frame_to_partition(STG_PARTITION_ANO, data, len,
					       first_frame);

	if (err) {
		LOG_ERR("Error writing to ano partition, err %i", err);
		k_mutex_unlock(&ano_mutex);
		return err;
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