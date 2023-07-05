#include "nclogs.h"
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
#include "seq_structure.h"
#include "pasture_structure.h"
#include "storage.h"
#include "storage_event.h"

#include "system_diagnostic_structure.h"

#include "error_event.h"

#include "messaging_module_events.h"

#include "embedded.pb.h"
#include "UBX.h"

#include <date_time.h>
#include <time.h>

/* Get sizes and offset definitions from pm_static.yml. */
#include <pm_config.h>

#include <logging/log.h>
#define MODULE storage_controller
LOG_MODULE_REGISTER(MODULE, CONFIG_STORAGE_CONTROLLER_LOG_LEVEL);

/* Seq partition. */
static const struct flash_area *seq_area;
static struct fcb seq_fcb;
static struct flash_sector seq_sectors[FLASH_SEQ_NUM_SECTORS];
static struct fcb_entry active_seq_entry = { .fe_sector = NULL, .fe_elem_off = 0 };
K_MUTEX_DEFINE(seq_mutex);

/* System diagnostic partition. */
static struct fcb_entry active_system_diag_entry = { .fe_sector = NULL, .fe_elem_off = 0 };
static const struct flash_area *system_diag_area;
static struct fcb system_diag_fcb;
static struct flash_sector system_diag_sectors[FLASH_SYSTEM_DIAG_NUM_SECTORS];
K_MUTEX_DEFINE(system_diag_mutex);

/* ANO partition. */
static const struct flash_area *ano_area;
static struct fcb ano_fcb;
static struct flash_sector ano_sectors[FLASH_ANO_NUM_SECTORS];
K_MUTEX_DEFINE(ano_mutex);

static struct fcb_entry last_sent_ano_entry = { .fe_sector = NULL, .fe_elem_off = 0 };

/* Pasture partition. */
static const struct flash_area *pasture_area;
static struct fcb pasture_fcb;
static struct flash_sector pasture_sectors[FLASH_PASTURE_NUM_SECTORS];
K_MUTEX_DEFINE(pasture_mutex);

K_KERNEL_STACK_DEFINE(erase_flash_thread, CONFIG_STORAGE_THREAD_SIZE);

static void erase_flash_fn(struct k_work *item);
static struct k_work_q erase_q;
static struct k_work erase_work;
static bool queue_inited = false;

void erase_flash_fn(struct k_work *item)
{
	NCLOG_INF(STORAGE_CONTROLLER, TRice0( iD( 7729),"inf: Erasing all flash partitions!\n"));
	int err = stg_clear_partition(STG_PARTITION_SEQ);
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
	err = stg_clear_partition(STG_PARTITION_SYSTEM_DIAG);
	if (err) {
		return;
	}

	struct flash_erased_event *ev = new_flash_erased_event();
	EVENT_SUBMIT(ev);
}

/** @brief Gets fcb structure based on partition.
 * 
 * @param partition which partition to get fcb from
 * 
 * @return pointer to fcb structure
 */
struct fcb *get_fcb(flash_partition_t partition)
{
	struct fcb *fcb;

	if (partition == STG_PARTITION_SEQ) {
		fcb = &seq_fcb;
	} else if (partition == STG_PARTITION_ANO) {
		fcb = &ano_fcb;
	} else if (partition == STG_PARTITION_PASTURE) {
		fcb = &pasture_fcb;
	} else if (partition == STG_PARTITION_SYSTEM_DIAG) {
		fcb = &system_diag_fcb;
	} else {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice0( iD( 6873),"err: Invalid partition given.\n"));
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
	if (partition == STG_PARTITION_SEQ) {
		return &seq_mutex;
	} else if (partition == STG_PARTITION_ANO) {
		return &ano_mutex;
	} else if (partition == STG_PARTITION_PASTURE) {
		return &pasture_mutex;
	} else if (partition == STG_PARTITION_SYSTEM_DIAG) {
		return &system_diag_mutex;
	}
	NCLOG_ERR(STORAGE_CONTROLLER, TRice0( iD( 6556),"err: Invalid partition given.\n"));
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
	if (partition == STG_PARTITION_SEQ) {
		sector_cnt = FLASH_SEQ_NUM_SECTORS;
		area_id = FLASH_AREA_ID(seq_partition);
		area = seq_area;
		sector_ptr = seq_sectors;
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
	} else if (partition == STG_PARTITION_SYSTEM_DIAG) {
		sector_cnt = FLASH_SYSTEM_DIAG_NUM_SECTORS;
		area_id = FLASH_AREA_ID(system_diagnostic);
		area = system_diag_area;
		sector_ptr = system_diag_sectors;
	} else {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 6412),"err: Invalid partition given. %d\n", -EINVAL));
		return -EINVAL;
	}

	err = flash_area_open(area_id, &area);
	if (err) {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 6773),"err: Error opening flash area for partition %d, err %d\n", partition, err));
		return err;
	}

	NCLOG_DBG(STORAGE_CONTROLLER, TRice( iD( 6657),"dbg: FCB Init: Partition%d, AreaID%d, FaID%d, FaOff%d, FaSize%d\n", partition, area_id, (uint8_t)area->fa_id, (int)area->fa_off, (int)area->fa_size));

	/* Check if area has a flash device available. */
	dev = device_get_binding(area->fa_dev_name);
	flash_area_close(area);

	if (dev == NULL) {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 1557),"err: Could not get device for partition %d.\n", partition));
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
		NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 4764),"err: Unable to setup sectors for partition %d, err %d.\n", partition, err));
		return err;
	}

	fcb->f_sector_cnt = (uint8_t)sector_cnt;
	fcb->f_sectors = sector_ptr;

	err = fcb_init(area_id, fcb);
	if (err) {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 7693),"err: Unable to initialize fcb for partition %d, err %d.\n", partition, err));
		return err;
	}

	NCLOG_INF(STORAGE_CONTROLLER, TRice( iD( 2802),"inf: Setup FCB for partition %d: %d sectors with sizes %db.\n", partition, fcb->f_sector_cnt, fcb->f_sectors[0].fs_size));

	return err;
}

int stg_init_storage_controller(void)
{
	int err;

	/* Give mutexes. First come first served. */
	k_mutex_unlock(&seq_mutex);
	k_mutex_unlock(&ano_mutex);
	k_mutex_unlock(&pasture_mutex);
	k_mutex_unlock(&system_diag_mutex);

	/* Initialize FCB on SEQ and ANO partitions
	 * based on pm_static.yml/.dts flash setup.
	 */
	err = init_fcb_on_partition(STG_PARTITION_SEQ);
	if (err) {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice(iD(3212), "err: STG_PARTITION_SEQ %d\n", err));
		return err;
	}

	err = init_fcb_on_partition(STG_PARTITION_ANO);
	if (err) {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice(iD(1425), "err: STG_PARTITION_ANO %d\n", err));
		return err;
	}

	err = init_fcb_on_partition(STG_PARTITION_PASTURE);
	if (err) {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice(iD(1487), "err: STG_PARTITION_PASTURE %d\n", err));
		return err;
	}

	err = init_fcb_on_partition(STG_PARTITION_SYSTEM_DIAG);
	if (err) {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice(iD(1608), "err: STG_PARTITION_SYSTEM_DIAG %d\n", err));
		return err;
	}

	/* Setup work threads. */
	if (!queue_inited) {
		k_work_queue_init(&erase_q);
		struct k_work_queue_config cfg = {
			.name = "storage_q",
		};
		k_work_queue_start(&erase_q, erase_flash_thread,
				   K_THREAD_STACK_SIZEOF(erase_flash_thread),
				   CONFIG_STORAGE_THREAD_PRIORITY, &cfg);
		k_work_init(&erase_work, erase_flash_fn);
		queue_inited = true;
	}

	return 0;
}

int stg_write_to_partition(flash_partition_t partition, uint8_t *data, size_t len)
{
	struct fcb_entry loc;
	struct fcb *fcb = get_fcb(partition);
	int err = 0;

	/* Check for multiple of 4 */
	int padding = 0;
	int multiple = (len % 4U);
	if (multiple != 0) {
		padding = 4 - multiple;
	}
	size_t new_len = len + padding;
	uint8_t *new_data = k_malloc(new_len);
	memset(new_data, 0xFF, new_len);
	memcpy(new_data, data, len);

	/* Appending a new entry, rotate(replaces) oldest if no space. */
	err = fcb_append(fcb, new_len, &loc);
	if (err == -ENOSPC) {
		err = fcb_rotate(fcb);
		if (err) {
			NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 2935),"err: Unable to rotate fcb from -ENOSPC, err %d\n", err));
			k_free(new_data);
			return err;
		}
		/* Retry appending. */
		err = fcb_append(fcb, new_len, &loc);
		if (err) {
			NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 3696),"err: Unable to recover in appending function, err %d\n", err));
			nf_app_error(ERR_STORAGE_CONTROLLER, -ENOTRECOVERABLE, NULL, 0);
			k_free(new_data);
			return err;
		}
		NCLOG_INF(STORAGE_CONTROLLER, TRice0( iD( 7014),"inf: Rotated FCB since it's full.\n"));
	} else if (err) {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 3281),"err: Error appending new fcb entry, err %d\n", err));
		k_free(new_data);
		return err;
	}

	err = flash_area_write(fcb->fap, FCB_ENTRY_FA_DATA_OFF(loc), new_data, new_len);
	if (err) {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 5041),"err: Error writing to flash area. err %d\n", err));
		k_free(new_data);
		return err;
	}

	/* Finish entry. */
	err = fcb_append_finish(fcb, &loc);
	if (err) {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 5985),"err: Error finishing new entry. err %d\n", err));
	}
	k_free(new_data);
	return err;
}

int stg_clear_partition(flash_partition_t partition)
{
	struct k_mutex *mtx = get_mutex(partition);

	if (mtx == NULL) {
		return -EINVAL;
	}

	if (k_mutex_lock(mtx, K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 6033),"err: Mutex timeout in storage controller when clearing FCB: Partition %i\n", partition));
		return -ETIMEDOUT;
	}

	if (partition == STG_PARTITION_ANO) {
		last_sent_ano_entry.fe_sector = NULL;
		last_sent_ano_entry.fe_elem_off = 0;
	} else if (partition == STG_PARTITION_SEQ) {
		active_seq_entry.fe_sector = NULL;
		active_seq_entry.fe_elem_off = 0;
	} else if (partition == STG_PARTITION_SYSTEM_DIAG) {
		active_system_diag_entry.fe_sector = NULL;
		active_system_diag_entry.fe_elem_off = 0;
	}

	struct fcb *fcb = get_fcb(partition);
	int err = fcb_clear(fcb);

	k_mutex_unlock(mtx);
	return err;
}

int stg_read_seq_data(fcb_read_cb cb, uint16_t num_entries)
{
	if (k_mutex_lock(&seq_mutex, K_NO_WAIT)) {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice0( iD( 2883),"err: locking seq_mutex failed. \n"));
	}
	if (seq_mutex.lock_count == 1) {
		if (fcb_is_empty(&seq_fcb)) {
			k_mutex_unlock(&seq_mutex);
			return -ENODATA;
		}

		struct fcb_entry start_entry;

		memcpy(&start_entry, &active_seq_entry, sizeof(struct fcb_entry));

		int err = fcb_getnext(&seq_fcb, &start_entry);
		if (err) {
			k_mutex_unlock(&seq_mutex);
			return -ENODATA;
		}

		err = fcb_walk_from_entry(cb, &seq_fcb, &start_entry, num_entries, &seq_mutex);
		/* Update the entry we're currently on. */
		memcpy(&active_seq_entry, &start_entry, sizeof(struct fcb_entry));

		if (err != 0) {
			NCLOG_ERR(STORAGE_CONTROLLER, TRice0( iD( 6467),"err: Error reading from seq partition.\n"));
			k_mutex_unlock(&seq_mutex);
			return err;
		}

		k_mutex_unlock(&seq_mutex);
		return err;
	} else {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice0( iD( 2874),"wrn: seq_mutex.lock_count != 1 \n"));
		k_mutex_unlock(&seq_mutex);
		return -EBUSY;
	}
}

int stg_read_ano_data(fcb_read_cb cb, bool read_from_start, uint16_t num_entries)
{
	if (k_mutex_lock(&ano_mutex, K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice0( iD( 3634),"wrn: ano_mutex timeout \n"));
		return -ETIMEDOUT;
	}

	int err = 0;

	if (fcb_is_empty(&ano_fcb)) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice0( iD( 1273),"wrn: No ano frames found on partition.\n"));
		k_mutex_unlock(&ano_mutex);
		return -ENODATA;
	}

	struct fcb_entry start_entry;

	if (read_from_start) {
		memset(&start_entry, 0, sizeof(start_entry));
	} else {
		memcpy(&start_entry, &last_sent_ano_entry, sizeof(struct fcb_entry));
	}

	err = fcb_getnext(&ano_fcb, &start_entry);
	if (err) {
		k_mutex_unlock(&ano_mutex);
		return -ENODATA;
	}

	err = fcb_walk_from_entry(cb, &ano_fcb, &start_entry, num_entries, &ano_mutex);

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
	if (k_mutex_lock(&pasture_mutex, K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		return -ETIMEDOUT;
	}

	struct fcb *fcb = get_fcb(STG_PARTITION_PASTURE);
	if (fcb_is_empty(fcb)) {
		k_mutex_unlock(&pasture_mutex);
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

	err = flash_area_read(fcb->fap, FCB_ENTRY_FA_DATA_OFF(entry), fence, fence_size);
	if (err) {
		k_free(fence);
		k_mutex_unlock(&pasture_mutex);
		return err;
	}

	err = cb(fence, fence_size);
	k_free(fence);
	k_mutex_unlock(&pasture_mutex);
	return err;
}

int stg_write_seq_data(uint8_t *data, size_t len)
{
	if (k_mutex_lock(&seq_mutex, K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT)) == 0 &&
	    seq_mutex.lock_count <= 1) {
		int err = stg_write_to_partition(STG_PARTITION_SEQ, data, len);
		if (err) {
			NCLOG_ERR(STORAGE_CONTROLLER, TRice0( iD( 6247),"err: Error writing to seq partition.\n"));
		}
		k_mutex_unlock(&seq_mutex);
		return err;
	} else {
		k_mutex_unlock(&seq_mutex);
		NCLOG_WRN(STORAGE_CONTROLLER, TRice0( iD( 6231),"wrn: seq_mutex timeout\n"));
		return -ETIMEDOUT;
	}
}

int stg_write_ano_data(const ano_rec_t *ano_rec)
{
	if (k_mutex_lock(&ano_mutex, K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		return -ETIMEDOUT;
	}

	int err = stg_write_to_partition(STG_PARTITION_ANO, (uint8_t *)ano_rec, sizeof(*ano_rec));

	if (err) {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 2444),"err: Error writing to ano partition, err %i\n", err));
	}

	k_mutex_unlock(&ano_mutex);
	return err;
}

int stg_write_pasture_data(uint8_t *data, size_t len)
{
	if (k_mutex_lock(&pasture_mutex, K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice0( iD( 1577),"wrn: pasture_mutex timeout\n"));
		return -ETIMEDOUT;
	}

	int err = stg_write_to_partition(STG_PARTITION_PASTURE, data, len);

	if (err) {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 3296),"err: Error writing to pasture partition %i\n", err));
	}

	k_mutex_unlock(&pasture_mutex);
	return err;
}

int stg_read_system_diagnostic_log(fcb_read_cb cb, uint16_t num_entries)
{
	if (k_mutex_lock(&system_diag_mutex, K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		return -ETIMEDOUT;
	}

	if (fcb_is_empty(&system_diag_fcb)) {
		k_mutex_unlock(&system_diag_mutex);
		return -ENODATA;
	}

	struct fcb_entry start_entry;

	memcpy(&start_entry, &active_system_diag_entry, sizeof(struct fcb_entry));

	int err = fcb_getnext(&system_diag_fcb, &start_entry);
	if (err) {
		k_mutex_unlock(&system_diag_mutex);
		return -ENODATA;
	}

	err = fcb_walk_from_entry(cb, &system_diag_fcb, &start_entry, num_entries,
				  &system_diag_mutex);
	if (err && err != -EINTR) {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice0( iD( 6957),"err: Error reading from system diagnostic partition.\n"));
	}

	/* Update the entry we're currently on. */
	memcpy(&active_system_diag_entry, &start_entry, sizeof(struct fcb_entry));

	k_mutex_unlock(&system_diag_mutex);
	return err;
}

int stg_write_system_diagnostic_log(uint8_t *data, size_t len)
{
	if (k_mutex_lock(&system_diag_mutex, K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		return -ETIMEDOUT;
	}

	int err = stg_write_to_partition(STG_PARTITION_SYSTEM_DIAG, data, len);

	if (err) {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 1561),"err: Error writing to system diagnostic partition %i\n", err));
	}

	k_mutex_unlock(&system_diag_mutex);
	return err;
}

uint32_t get_num_entries(flash_partition_t partition)
{
	struct fcb *fcb = get_fcb(partition);
	struct k_mutex *mtx = get_mutex(partition);

	if (mtx == NULL || fcb == NULL) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice0( iD( 4604),"wrn: partition invalid \n"));
		return -EINVAL;
	}

	if (k_mutex_lock(mtx, K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice0( iD( 3225),"err: Mutex timeout in storage controller when clearing.\n"));
		return -ETIMEDOUT;
	}

	struct fcb_entry oldest_entry;

	memset(&oldest_entry, 0, sizeof(struct fcb_entry));
	oldest_entry.fe_sector = NULL;

	uint32_t cnt = 0;
	while (fcb_getnext(fcb, &oldest_entry) == 0) {
		cnt++;
	}

	k_mutex_unlock(mtx);
	return cnt;
}

int stg_fcb_reset_and_init()
{
	memset(&seq_fcb, 0, sizeof(seq_fcb));
	memset(&ano_fcb, 0, sizeof(ano_fcb));
	memset(&pasture_fcb, 0, sizeof(pasture_fcb));
	memset(&system_diag_fcb, 0, sizeof(pasture_fcb));

	memset(&last_sent_ano_entry, 0, sizeof(struct fcb_entry));
	memset(&active_seq_entry, 0, sizeof(struct fcb_entry));
	memset(&active_system_diag_entry, 0, sizeof(struct fcb_entry));

	return stg_init_storage_controller();
}

bool stg_seq_pointing_to_last()
{
	if (k_mutex_lock(&seq_mutex, K_MSEC(CONFIG_MUTEX_READ_WRITE_TIMEOUT))) {
		return false;
	}

	if (fcb_is_empty(&seq_fcb)) {
		k_mutex_unlock(&seq_mutex);
		return false;
	}

	struct fcb_entry test_entry;
	memcpy(&test_entry, &active_seq_entry, sizeof(struct fcb_entry));

	if (fcb_getnext(&seq_fcb, &test_entry) != 0) {
		k_mutex_unlock(&seq_mutex);
		return true;
	}

	k_mutex_unlock(&seq_mutex);
	return false;
}

static bool event_handler(const struct event_header *eh)
{
	if (is_request_flash_erase_event(eh)) {
		struct request_flash_erase_event *ev = cast_request_flash_erase_event(eh);

		if (ev->magic == STORAGE_ERASE_MAGIC) {
			k_work_submit_to_queue(&erase_q, &erase_work);
		}
		return true;
	}

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, request_flash_erase_event);
