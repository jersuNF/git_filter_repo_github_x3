#include <zephyr.h>
#include <dfu/dfu_target.h>
#include <dfu/dfu_target_mcuboot.h>
#include <dfu/dfu_target_stream.h>
#include <dfu/mcuboot.h>
#include "fw_upgrade_events.h"
#include "fw_upgrade.h"
#include <power/reboot.h>

/* Variable to store the current progress of DFU. Used by the
module to determine if we want to initialize DFU library (dfu_bytes_written = 0), 
or we want to schedule a reboot since we're finished (dfu_bytes_written = file_size) */
static uint64_t dfu_bytes_written = 0;

/* Buffer for mcuboot DFU */
static uint8_t mcuboot_buf[CONFIG_DFU_MCUBOOT_FLASH_BUF_SZ] __aligned(4);

/* Delayable work queue to schedule reboot */
struct k_work_delayable reboot_device_work;

static void reboot_device_fn(struct k_work *item)
{
	sys_reboot(SYS_REBOOT_COLD);
}

int fw_upgrade_module_init()
{
	/* Start by setting status event to idle, nothing in progress */
	struct dfu_status_event *event = new_dfu_status_event();
	event->trigger_type = DFU_TRIGGER_TYPE_IDLE;
	event->dfu_status = DFU_STATUS_IDLE;

	/* Submit event. */
	EVENT_SUBMIT(event);

	/* Initialize the reboot work function */
	k_work_init_delayable(&reboot_device_work, reboot_device_fn);

	/* Set byte written to 0 and clean dfu resources */
	dfu_bytes_written = 0;

	/* Reset all DFU resources to start a clean upgrade */
	int err = 0;
	err = dfu_target_reset();
	if (err) {
		//LOG_INF("dft_target_reset err: %d", err);
	}
	return err;
}

/**
 * @brief Callback function for device firmware upgrade during the process. 
 * This function is called with a return value if something happens during the write
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
 * @brief Main function for applying fragments to internal flash, and then performing a reboot
 * by scheduling after N seconds and updating the event so we can shutdown modules if needed before
 * the reboot.
 * @param fragment pointer to first byte of the fragment chunk
 * @param fragment_size size of the fragment received, this size can vary
 * @param file_size size of the firmware file we want to upgrade, used to compare
 * how far we've come in the process, so that we can initialize the first fragment correctly
 * and finish the upgrade correctly.
 * @param trigger_type what triggered the firmware upgrade, bluetooth or the modem?
 * @return 0 on success, otherwise negative errno
 */
static inline int apply_fragment(uint8_t *fragment, size_t fragment_size,
				 size_t file_size,
				 enum dfu_trigger_type trigger_type)
{
	int err;

	/* Check if we just started DFU process, pass further to DFU support library */
	if (dfu_bytes_written == 0) {
		/* Reset all DFU resources to start a clean upgrade */
		err = dfu_target_reset();
		if (err) {
			//LOG_INF("dft_target_reset err: %d", err);
		}

		//LOG_INF("Starting DFU procedure...");
		err = dfu_target_mcuboot_set_buf(mcuboot_buf,
						 sizeof(mcuboot_buf));
		if (err) {
			//LOG_ERR("Failed to set MCUboot flash buffer %d", err);
			return err;
		}
		int result = dfu_target_img_type(fragment, fragment_size);
		if (result < 0) {
			//LOG_ERR("Error selecting image type... %i", result);
			return result;
		}
		/* Initialize dfu target, by linking callback function for further error handling */
		err = dfu_target_init(result, file_size, dfu_apply_cb);
		if (err) {
			//LOG_ERR("Error initing dfu target... %i", err);
			return err;
		};
		//LOG_INF("Selected image type %i", result);

		/* Init successful, meaning we can update status event that DFU is in progress */
		struct dfu_status_event *dfu_event_in_progress =
			new_dfu_status_event();
		dfu_event_in_progress->trigger_type = trigger_type;
		dfu_event_in_progress->dfu_status = DFU_STATUS_IN_PROGRESS;

		/* Submit event. */
		EVENT_SUBMIT(dfu_event_in_progress);
	}

	/* Init performed if it was the first fragment. Write the fragment to internal flash using dfu library */
	err = dfu_target_write(fragment, fragment_size);
	if (err) {
		//LOG_ERR("Error writing dfu target... %i", err);
		dfu_target_done(false);
		dfu_bytes_written = 0;
		return err;
	} else {
		dfu_bytes_written += fragment_size;
		//LOG_INF("Wrote %d bytes of %d bytes for DFU process", dfu_bytes_written, file_size);
	}

	/* Everything went through with the fragment, check if it was the last fragment*/
	if (dfu_bytes_written == file_size) {
		err = dfu_target_done(true);
		if (err) {
			//LOG_ERR("Couldn't deinit dfu resources after succeeded dfu proccess.. %i", err);
			return err;
		}
		/* Will update status event that DFU finished, modules subscribing will then 
            have time to shutdown correctly. Also schedule reboot work item after n seconds */
		/* Create a status event if we need to update progress, finished etc */
		struct dfu_status_event *dfu_event_done =
			new_dfu_status_event();
		dfu_event_done->trigger_type = trigger_type;
		dfu_event_done->dfu_status =
			DFU_STATUS_SUCCESS_REBOOT_SCHEDULED;

		/* Submit event. */
		EVENT_SUBMIT(dfu_event_done);
		k_work_reschedule(&reboot_device_work,
				  K_SECONDS(CONFIG_SCHEDULE_REBOOT_SECONDS));
		/* Set bytes written to 0 if we need to restart upgrade if reboot failed*/
		dfu_bytes_written = 0;
	} else if (dfu_bytes_written > file_size) {
		/* Something happened with the DFU process, cannot write more bytes than file_size;
        handle error accordingly here...*/
		return -EMSGSIZE;
	}
	return 0;
}

/**
 * @brief Main event handler function. This simply checks if the firmware fragment received event
 * is updated, in which case we just forward the fragment data to function above (apply_fragment).
 * @param eh event_header for the if-chain to use to recognize which event triggered
 */
static bool event_handler(const struct event_header *eh)
{
	int err;
	if (is_dfu_fragment_event(eh)) {
		struct dfu_fragment_event *event = cast_dfu_fragment_event(eh);

		/* Call function that writes given fragment to internal flash S1 */
		err = apply_fragment((uint8_t *)&event->fragment,
				     event->fragment_size, event->file_size,
				     event->trigger_type);
		// Error handling in the modules??

		/* Consume event and wait for next update*/
		return true;
	}
	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, dfu_fragment_event);