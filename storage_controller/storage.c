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
#include "storage_events.h"
#include <fs/fcb.h>

#include "ano_structure.h"
#include "log_structure.h"
#include "pasture_structure.h"
#include "storage.h"

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

/* Semaphore used by storage controller to continue the walk process only
 * when this semaphore was given in the consumed event sent out by the
 * consumer/requester.
 */
K_SEM_DEFINE(data_consumed_sem, 0, 1);

static volatile bool has_inited = false;

#define NUM_STG_MSGQ_EVENTS 2
/* 4 means 4-byte alignment. */
K_MSGQ_DEFINE(read_event_msgq, sizeof(struct stg_read_event),
	      CONFIG_MSGQ_READ_EVENT_SIZE, 4);
K_MSGQ_DEFINE(write_event_msgq, sizeof(struct stg_write_event),
	      CONFIG_MSGQ_WRITE_EVENT_SIZE, 4);

struct k_poll_event stg_msgq_events[NUM_STG_MSGQ_EVENTS] = {
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
					K_POLL_MODE_NOTIFY_ONLY,
					&read_event_msgq, 0),
	K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
					K_POLL_MODE_NOTIFY_ONLY,
					&write_event_msgq, 0),
};

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

int init_storage_controller(void)
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

void stg_fcb_write_entry()
{
	bool rotated = false;
	struct stg_write_event ev;
	int err = k_msgq_get(&write_event_msgq, &ev, K_NO_WAIT);
	if (err) {
		LOG_ERR("Error getting write_event from msgq: %d", err);
		return;
	}

	struct fcb_entry loc;
	struct fcb *fcb = get_fcb(ev.partition);

	/* If we want to only read the newest, clear all previous entries. */
	if (ev.rotate_to_this) {
		err = fcb_clear(fcb);
		if (err) {
			return;
		}
	}

	/* Appending a new entry, rotate(replaces) oldests if no space. */
	err = fcb_append(fcb, ev.len, &loc);
	if (err == -ENOSPC) {
		err = fcb_rotate(fcb);
		if (err) {
			LOG_ERR("Unable to rotate fcb from -ENOSPC, err %d",
				err);
			return;
		}
		/* Retry appending. */
		err = fcb_append(fcb, ev.len, &loc);
		if (err) {
			LOG_ERR("Unable to recover in appending function, err %d",
				err);
			nf_app_error(ERR_SENDER_STORAGE_CONTROLLER,
				     -ENOTRECOVERABLE, NULL, 0);
			return;
		}
		LOG_INF("Rotated FCB since it's full.");
		rotated = true;
	} else if (err) {
		LOG_ERR("Error appending new fcb entry, err %d", err);
		return;
	}

	err = flash_area_write(fcb->fap, FCB_ENTRY_FA_DATA_OFF(loc), ev.data,
			       ev.len);
	if (err) {
		LOG_ERR("Error writing to flash area. err %d", err);
		return;
	}

	/* Finish entry. */
	err = fcb_append_finish(fcb, &loc);
	if (err) {
		LOG_ERR("Error finishing new entry. err %d", err);
		return;
	}

	/* Notify data has been written. */
	struct stg_ack_write_event *event = new_stg_ack_write_event();
	event->partition = ev.partition;
	event->rotated = rotated;
	EVENT_SUBMIT(event);
	return;
}

/** @brief Helper function for reading entry from external flash and then
 *         using malloc to cache the data and forward it to an output
 *         data buffer and length. Free's the data when consume event
 *         has been received by this module, or if it times out.
 * 
 * @param[in] partition which partition to target in the read_ack.
 * @param[in] fa flash_area of which area on the flash is used. (partition)
 * @param[in] loc fcb_entry which contains information of the contents on
 *                external flash.
 * 
 * @return 0 on success, otherwise negative errno.
 */
static inline int stg_read_entry_raw(flash_partition_t partition, size_t len,
				     off_t offset)
{
	struct stg_ack_read_event *ev_ack = new_stg_ack_read_event();
	struct fcb *fcb = get_fcb(partition);

	int err = 0;

	/* Allocate space so the requester can access this region. */
	uint8_t *data = (uint8_t *)k_malloc(len);

	if (data == NULL) {
		LOG_ERR("No heap memory to allocate temporary data buffer.");
		return -ENOMEM;
	}

	err = flash_area_read(fcb->fap, offset, data, len);
	if (err) {
		LOG_ERR("Error reading from flash area, err %d", err);
		goto cleanup;
	}

	/* Entry now read, tell requester the data is ready for consumption. */

	ev_ack->partition = partition;
	ev_ack->data = data;
	ev_ack->len = len;

	EVENT_SUBMIT(ev_ack);

	err = k_sem_take(&data_consumed_sem,
			 K_SECONDS(CONFIG_WALK_CONSUME_TIMEOUT_SEC));
	if (err) {
		LOG_ERR("Failed to wait for requester to consume %d", err);
		goto cleanup;
	}
cleanup:
	k_free(data);
	return err;
}

static int stg_fcb_walk_cb(struct fcb_entry_ctx *loc_ctx, void *arg)
{
	struct stg_read_event *ev_read = (struct stg_read_event *)arg;

	off_t offset = FCB_ENTRY_FA_DATA_OFF(loc_ctx->loc);
	size_t len = loc_ctx->loc.fe_data_len;

	return stg_read_entry_raw(ev_read->partition, len, offset);
}

int clear_fcb_sectors(flash_partition_t partition)
{
	struct fcb *fcb = get_fcb(partition);
	return fcb_clear(fcb);
}

void stg_fcb_read_entry()
{
	struct stg_read_event ev;
	int err = k_msgq_get(&read_event_msgq, &ev, K_NO_WAIT);
	if (err) {
		LOG_ERR("Error getting read_event from msgq: %d", err);
		return;
	}

	struct fcb *fcb = get_fcb(ev.partition);

	err = fcb_walk(fcb, NULL, stg_fcb_walk_cb, &ev);
	if (err) {
		LOG_ERR("Error walking over FCB storage, err %d", err);
		return;
	}

	if (ev.rotate) {
		err = fcb_clear(fcb);
		if (err) {
			LOG_ERR("Error clearing FCB after walk, err %d", err);
			return;
		}
	}
	return;
}

int stg_fcb_reset_and_init()
{
	memset(&log_fcb, 0, sizeof(log_fcb));
	memset(&ano_fcb, 0, sizeof(ano_fcb));
	memset(&pasture_fcb, 0, sizeof(pasture_fcb));

	return init_storage_controller();
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
	/* If storage module is not initialized and FCBs are not set up, just
	 * consume event and let the requester hang to indicate that
	 * they have to try again after this module is initialized.
	 */
	if (!has_inited) {
		return false;
	}
	if (is_stg_read_event(eh)) {
		struct stg_read_event *ev = cast_stg_read_event(eh);
		while (k_msgq_put(&read_event_msgq, ev, K_NO_WAIT) != 0) {
			k_msgq_purge(&read_event_msgq);
		}
		return false;
	}
	if (is_stg_write_event(eh)) {
		struct stg_write_event *ev = cast_stg_write_event(eh);
		while (k_msgq_put(&write_event_msgq, ev, K_NO_WAIT) != 0) {
			k_msgq_purge(&write_event_msgq);
		}
		return false;
	}
	if (is_stg_consumed_read_event(eh)) {
		k_sem_give(&data_consumed_sem);
		return false;
	}

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(MODULE, event_handler);

EVENT_SUBSCRIBE(MODULE, stg_write_event);
EVENT_SUBSCRIBE(MODULE, stg_read_event);
EVENT_SUBSCRIBE(MODULE, stg_consumed_read_event);

void storage_thread_fn()
{
	while (true) {
		int err =
			k_poll(stg_msgq_events, NUM_STG_MSGQ_EVENTS, K_FOREVER);
		if (err == 0) {
			while (k_msgq_num_used_get(&read_event_msgq) > 0) {
				stg_fcb_read_entry();
			}
			while (k_msgq_num_used_get(&write_event_msgq) > 0) {
				stg_fcb_write_entry();
			}
		}
	}
	/* Set all the events to not ready again. */
	for (int i = 0; i < NUM_STG_MSGQ_EVENTS; i++) {
		stg_msgq_events[i].state = K_POLL_STATE_NOT_READY;
	}
}

/* Thread stack area that we use for the storage controller operations 
 * such as read/write.
 */
K_THREAD_DEFINE(storage_thread, CONFIG_STORAGE_THREAD_SIZE, storage_thread_fn,
		NULL, NULL, NULL, CONFIG_STORAGE_THREAD_PRIORITY, 0, 0);