/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include <stdio.h>

#include "fw_upgrade_events.h"
#include "fw_upgrade.h"
#include <logging/log.h>

#include "error_event.h"

#include <dfu/mcuboot.h>
#include <dfu/dfu_target_mcuboot.h>
#include <net/fota_download.h>

#include "pwr_event.h"

#define MODULE fw_upgrade
LOG_MODULE_REGISTER(MODULE, CONFIG_FW_UPGRADE_LOG_LEVEL);

#define CACHE_HOST_NAME "172.31.36.11:5252"

/** @brief Make sure we give the FOTA path to the hardware we built on. */
#ifdef CONFIG_BOARD_NF_SG25_27O_NRF52840
#define CACHE_PATH_NAME "firmware/x25/%i/nf_sg25_27o_nrf52840/app_update.bin"
#elif CONFIG_BOARD_NF_C25_25G_NRF52840
#define CACHE_PATH_NAME "firmware/x25/%i/nf_c25_25g_nrf52840/app_update.bin"
#elif CONFIG_BOARD_NATIVE_POSIX
#define CACHE_PATH_NAME "firmware/x25/%i/not_valid!/app_update.bin"
#else
#error Unsupported boardfile for performing Nofence FOTA! (SG25/C25 only)
#endif

static void fota_dl_handler(const struct fota_download_evt *evt)
{
	/* Start by setting status event to idle, nothing in progress. */
	switch (evt->id) {
	case FOTA_DOWNLOAD_EVT_ERROR: {
		struct dfu_status_event *event = new_dfu_status_event();
		LOG_ERR("Received error from fota_download %d", evt->cause);
		char *e_msg = "Fota download error";
		nf_app_error(ERR_FW_UPGRADE, -evt->cause, e_msg, strlen(e_msg));

		event->dfu_status = DFU_STATUS_IDLE;
		event->dfu_error = evt->cause;
		/* Submit event. */
		EVENT_SUBMIT(event);

		break;
	}
	case FOTA_DOWNLOAD_EVT_FINISHED: {
		struct dfu_status_event *event = new_dfu_status_event();
		LOG_INF("Fota download finished, scheduling reboot...");

		event->dfu_status = DFU_STATUS_SUCCESS_REBOOT_SCHEDULED;
		event->dfu_error = 0;

		/* Submit event. */
		EVENT_SUBMIT(event);

		struct pwr_reboot_event *reboot_ev = new_pwr_reboot_event();
		reboot_ev->reason = REBOOT_FOTA_RESET;
		EVENT_SUBMIT(reboot_ev);
		break;
	}
	default:
		break;
	}
}

int fw_upgrade_module_init()
{
	/* Start by setting status event to idle, nothing in progress. */
	struct dfu_status_event *event = new_dfu_status_event();

	event->dfu_status = DFU_STATUS_IDLE;
	event->dfu_error = 0;

	/* Submit event. */
	EVENT_SUBMIT(event);

	return fota_download_init(fota_dl_handler);
}

void mark_new_application_as_valid(void)
{
	/* Will return error code, but we do not need to do anything with it
	 * as we most likely would just revert to old firmware anyways.
	 */
	int err = boot_write_img_confirmed();
	if (err) {
		LOG_ERR("Error marking the new firmware as valid. err %d", err);
	}
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
	if (is_start_fota_event(eh)) {
		struct start_fota_event *ev = cast_start_fota_event(eh);

		char host_tmp[CONFIG_FW_UPGRADE_HOST_LEN];
		char path_tmp[CONFIG_FW_UPGRADE_PATH_LEN];

		if (ev->override_default_host) {
			memcpy(host_tmp, ev->host, CONFIG_FW_UPGRADE_HOST_LEN);
			memcpy(path_tmp, ev->path, CONFIG_FW_UPGRADE_PATH_LEN);
		} else {
			memcpy(host_tmp, CACHE_HOST_NAME,
			       sizeof(CACHE_HOST_NAME));
			snprintf(path_tmp, CONFIG_FW_UPGRADE_PATH_LEN,
				 CACHE_PATH_NAME, ev->version);
		}

		/* If no error, submit in progress event. */
		if (!fota_download_start(host_tmp, path_tmp, -1, 0, 0)) {
			struct dfu_status_event *status =
				new_dfu_status_event();

			status->dfu_status = DFU_STATUS_IN_PROGRESS;
			status->dfu_error = 0;

			EVENT_SUBMIT(status);
		}

		return false;
	}
	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, start_fota_event);
