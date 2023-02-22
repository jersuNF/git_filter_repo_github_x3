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

#define _MAKE_CACHE_PATH_NAME(a) "firmware/x25/%u/" #a "/app_update.bin?s=%u"

/** @brief Make sure we give the FOTA path to the hardware we built on. */
#ifdef CONFIG_BOARD_NF_SG25_27O_NRF52840
#define CACHE_PATH_NAME _MAKE_CACHE_PATH_NAME(nf_sg25_27o_nrf52840)
#elif CONFIG_BOARD_NF_C25_25G_NRF52840
#define CACHE_PATH_NAME _MAKE_CACHE_PATH_NAME(nf_c25_25g_nrf52840)
#elif CONFIG_BOARD_NATIVE_POSIX
#define CACHE_PATH_NAME _MAKE_CACHE_PATH_NAME(NOT_VALID)
#else
#error Unsupported boardfile for performing Nofence FOTA! (SG25/C25 only)
#endif

/* %u -> 4294967295 => 2 bytes-> 10 bytes, i.e. increase of 16 bytes  */
#define CACHE_PATH_NAME_MAX_LEN sizeof(CACHE_PATH_NAME) + 16

#define FOTA_RETRIES                                                                               \
	2 /* to ensure modem is switched back to PSV in case of
 * unhandled FOTA download errors */
static bool cancel_fota = false;

/* This variable MUST be static, as the FOTA subsystem stores a pointer to it */
static char host_tmp[CONFIG_FW_UPGRADE_HOST_LEN + sizeof(":1234")];
/* This variable MUST be static, as the FOTA subsystem stores a pointer to it */
static char path_tmp[CACHE_PATH_NAME_MAX_LEN + 1];

K_SEM_DEFINE(dl_client_offload, 0, 1);
struct k_thread start_download_client_thread;
K_THREAD_STACK_DEFINE(start_download_client_stack, CONFIG_START_DL_CLIENT_STACK);

static void attempt_to_start_dl_client(void)
{
	while (true) {
		if (k_sem_take(&dl_client_offload, K_FOREVER) == 0) {
			if (cancel_fota) {
				int ret = fota_download_cancel();
				if (ret == -EAGAIN) {
					LOG_WRN("LTE FOTA is not running");
				} else if (ret != 0) {
					LOG_ERR("Failed to cancel FOTA request");
				}
			} else {
				/* If no error, submit in progress event. */
				int ret = fota_download_start(host_tmp, path_tmp, -1, 0, 0);
				if (ret == 0) {
					struct dfu_status_event *status = new_dfu_status_event();
					status->dfu_status = DFU_STATUS_IN_PROGRESS;
					status->dfu_error = 0;
					EVENT_SUBMIT(status);
				} else if (ret == -EALREADY) {
					/* If FOTA is already in process */
					struct dfu_status_event *status = new_dfu_status_event();
					status->dfu_status = DFU_STATUS_ALREADY_RUNNING;
					status->dfu_error = -EALREADY;
					EVENT_SUBMIT(status);
				} else {
					/* Any other error means the DFU is now idle */
					struct dfu_status_event *status = new_dfu_status_event();
					status->dfu_status = DFU_STATUS_IDLE;
					status->dfu_error = -ENODATA;
					EVENT_SUBMIT(status);
				}
			}
		}
	}
}

static void fota_dl_handler(const struct fota_download_evt *evt)
{
	/* Start by setting status event to idle, nothing in progress. */
	switch (evt->id) {
	case FOTA_DOWNLOAD_EVT_PROGRESS: {
		struct dfu_status_event *event = new_dfu_status_event();
		event->dfu_status = DFU_STATUS_IN_PROGRESS;
		EVENT_SUBMIT(event);
		break;
	}
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
	case FOTA_DOWNLOAD_EVT_CANCELLED: {
		struct dfu_status_event *event = new_dfu_status_event();
		LOG_WRN("Fota download cancelled");
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

	k_thread_create(&start_download_client_thread, start_download_client_stack,
			K_KERNEL_STACK_SIZEOF(start_download_client_stack),
			(k_thread_entry_t)attempt_to_start_dl_client, NULL, NULL, NULL,
			CONFIG_DOWNLOAD_CLIENT_OFFLOAD_THREAD_PRIO, 0, K_NO_WAIT);

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

		/* @todo, the code below needs refactoring */
		memset(path_tmp, 0, sizeof(path_tmp));
		memset(host_tmp, 0, sizeof(host_tmp));

		if (ev->override_default_host) {
			snprintf(host_tmp, sizeof(host_tmp), "%s:%d", ev->host, 5252);
		} else {
			strncpy(host_tmp, CACHE_HOST_NAME, sizeof(host_tmp) - 1);
		}
		/* We are passing the serial_id in the HTTP request so that the server
		 * easily can trace or download
		 */
		snprintf(path_tmp, sizeof(path_tmp), CACHE_PATH_NAME, ev->version, ev->serial_id);
		cancel_fota = false;
		k_sem_give(&dl_client_offload);

		return false;
	}
	if (is_cancel_fota_event(eh)) {
		/* Cancel an ongoing FOTA. This will trigger FOTA_DOWNLOAD_EVT_CANCELLED 
		 * status in the fota_dl_handler callback 
		 */
		cancel_fota = true;
		k_sem_give(&dl_client_offload);
		return false;
	}

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, start_fota_event);
EVENT_SUBSCRIBE(MODULE, cancel_fota_event);
