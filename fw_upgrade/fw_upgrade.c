/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include <dfu/dfu_target.h>
#include <dfu/dfu_target_mcuboot.h>
#include <dfu/dfu_target_stream.h>
#include <dfu/mcuboot.h>
#include "fw_upgrade_events.h"
#include "fw_upgrade.h"
#include <power/reboot.h>
#include <logging/log.h>
#include "http_downloader.h"

#define LOG_MODULE_NAME fw_upgrade
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_FW_UPGRADE_LOG_LEVEL);

/* Variable to store the current progress of DFU. Used by the
 * module to determine if we want to initialize 
 * DFU library (dfu_bytes_written = 0), 
 * or we want to schedule a reboot 
 * since we're finished (dfu_bytes_written = file_size).
 */
static volatile uint32_t dfu_bytes_written = 0;

/* Buffer for mcuboot DFU. */
static uint8_t mcuboot_buf[CONFIG_DFU_MCUBOOT_FLASH_BUF_SZ] __aligned(4);

struct k_work_delayable reboot_device_work;

static void reboot_device_fn(struct k_work *item)
{
	sys_reboot(SYS_REBOOT_COLD);
}

int fw_upgrade_module_init()
{
	/* Start by setting status event to idle, nothing in progress. */
	struct dfu_status_event *event = new_dfu_status_event();
	//event->trigger_type = DFU_TRIGGER_TYPE_IDLE;
	event->dfu_status = DFU_STATUS_IDLE;

	/* Submit event. */
	EVENT_SUBMIT(event);

	/* Initialize the reboot work function. */
	k_work_init_delayable(&reboot_device_work, reboot_device_fn);

	/* Set byte written to 0 and clean dfu resources. */
	dfu_bytes_written = 0;

	/* Reset all DFU resources to start a clean upgrade. */
	int err = 0;
	err = dfu_target_reset();
	if (err) {
		LOG_INF("dft_target_reset err: %i", err);
	}

	/* Initialize the downloader module. */
	err = http_download_init();
	return err;
}

/**
 * @brief Callback function to handle DFU events if needed later.
 *
 * @param[in] evt event type that triggered the callback.
 */
static void dfu_apply_cb(enum dfu_target_evt_id evt)
{
	switch (evt) {
	case DFU_TARGET_EVT_TIMEOUT:
		break;
	case DFU_TARGET_EVT_ERASE_DONE:
		break;
	default:
		break;
	}
}

/**
 * @brief Main function for applying fragments to internal flash, 
 *        and then performing a reboot by scheduling after N seconds 
 *        and updating the event to handle modules if needed before the reboot.
 * 
 * @param[in] fragment Pointer to first byte of the fragment chunk.
 * @param[in] fragment_size Size of the fragment received, this size can vary.
 * @param[in] file_size Size of the firmware file we want to upgrade, 
 *                      used to compare how far we've come in the process, 
 *                      so that we can initialize the first fragment correctly
 *                      and finish the upgrade.
 * 
 * @return 0 on success. Otherwise a negative error code.
 */
static inline int apply_fragment(uint8_t *fragment, size_t fragment_size,
				 size_t file_size)
{
	int err;

	/* Check if we just started DFU process, 
	 * pass further to DFU support library.
	 */
	if (dfu_bytes_written == 0) {
		/* Reset all DFU resources to start a clean upgrade. */
		err = dfu_target_reset();
		if (err) {
			LOG_INF("dft_target_reset err: %i", err);
		}

		LOG_INF("Starting DFU procedure...");
		err = dfu_target_mcuboot_set_buf(mcuboot_buf,
						 sizeof(mcuboot_buf));
		if (err) {
			LOG_ERR("Failed to set MCUboot flash buffer %i", err);
			goto error_cleanup;
		}
		int result = dfu_target_img_type(fragment, fragment_size);
		if (result < 0) {
			LOG_ERR("Error selecting image type... %i", result);
			err = result;
			goto error_cleanup;
		}
		/* Initialize dfu target, by linking callback 
		 * function for further error handling. 
		 */
		err = dfu_target_init(result, file_size, dfu_apply_cb);
		if (err) {
			LOG_ERR("Error initing dfu target... %i", err);
			goto error_cleanup;
		};
		LOG_INF("Selected image type %i", result);

		/* Init successful, meaning we can update status 
		 * event that DFU is in progress. 
		 */
		struct dfu_status_event *dfu_event_in_progress =
			new_dfu_status_event();
		dfu_event_in_progress->dfu_status = DFU_STATUS_IN_PROGRESS;
		dfu_event_in_progress->dfu_error = 0;

		/* Submit event. */
		EVENT_SUBMIT(dfu_event_in_progress);
	}

	/* Init performed if it was the first fragment. 
	 * Write the fragment to internal flash using dfu library. 
	 */
	err = dfu_target_write(fragment, fragment_size);
	if (err) {
		LOG_ERR("Error writing dfu target... %i", err);
		dfu_target_done(false);
		dfu_bytes_written = 0;
		goto error_cleanup;
	} else {
		dfu_bytes_written += fragment_size;
		LOG_INF("Wrote %i bytes of %i bytes for DFU process",
			dfu_bytes_written, file_size);
	}

	/* Everything went through with the fragment, 
	 * and check if it was the last fragment.
	 */
	if (dfu_bytes_written == file_size) {
		err = dfu_target_done(true);
		if (err) {
			LOG_ERR("Couldn't deinit dfu resources \
			after succeeded dfu proccess.. %i",
				err);
			goto error_cleanup;
		}

		/* Will update status event that DFU finished, 
		 * modules subscribing will then have time to shutdown correctly. 
		 * Also schedule reboot work item after n seconds.
		 */
		struct dfu_status_event *dfu_event_done =
			new_dfu_status_event();

		dfu_event_done->dfu_status =
			DFU_STATUS_SUCCESS_REBOOT_SCHEDULED;
		dfu_event_done->dfu_error = 0;

		EVENT_SUBMIT(dfu_event_done);

		k_work_reschedule(&reboot_device_work,
				  K_SECONDS(CONFIG_SCHEDULE_REBOOT_SECONDS));

		/* Set to 0 so we're able to re-trigger 
		 * upgrade if reboot failed. 
		 */
		dfu_bytes_written = 0;
	} else if (dfu_bytes_written > file_size) {
		/* Received more fragment data than 
		 * what the file_size told us. 
		 */
		err = -EMSGSIZE;
		goto error_cleanup;
	}
	return 0;
error_cleanup:
	/* Make sure that all errors reset the 
	 * bytes-currently-written counter
	 * so that we at any time can re-trigger firmware upgrade from
	 * the start offset.
	 */
	dfu_bytes_written = 0;
	return err;
}

/**
 * @brief Main event handler function. 
 *        This simply checks if the firmware fragment received event
 *        is updated, in which case we just forward 
 *        the fragment data to function apply_fragment.
 * 
 * @param[in] eh Event_header for the if-chain to 
 *               use to recognize which event triggered.
 * 
 * @return True or false based on if we want to consume the event or not.
 */
static bool event_handler(const struct event_header *eh)
{
	int err;
	if (is_dfu_fragment_event(eh)) {
		struct dfu_fragment_event *event = cast_dfu_fragment_event(eh);

		/* Call function that writes 
		 * given fragment to internal flash S1. 
		 */
		err = apply_fragment((uint8_t *)&event->dyndata.data,
				     event->dyndata.size, event->file_size);
		/* Handle errors on status event. */
		if (err) {
			struct dfu_status_event *dfu_event_error =
				new_dfu_status_event();

			dfu_event_error->dfu_error = err;

			EVENT_SUBMIT(dfu_event_error);
		}
		/* Consume event and wait for next update. */
		return false;
	}
	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, dfu_fragment_event);