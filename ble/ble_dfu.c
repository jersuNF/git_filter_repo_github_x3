/*
 * Copyright (c) 2021 Nofence AS
 */

#include <init.h>

#include <img_mgmt/img_mgmt.h>
#include <os_mgmt/os_mgmt.h>
#include <mgmt/mcumgr/smp_bt.h>

#include <logging/log.h>
#include "fw_upgrade_events.h"

LOG_MODULE_REGISTER(ble_dfu);

static void dfu_started_cb(void)
{
	LOG_INF("BLE DFU upload started");

	/* Since BLE FOTA takes precedence, block incomming and cancel ongoing LTE FOTA */
	struct block_fota_event *block_ev = new_block_fota_event();
	block_ev->block_lte_fota = true;
	EVENT_SUBMIT(block_ev);

	struct cancel_fota_event *cancel_ev = new_cancel_fota_event();
	EVENT_SUBMIT(cancel_ev);
}

static void dfu_stopped_cb(void)
{
	LOG_ERR("BLE DFU process went wrong.");

	/* Unblock LTE FOTA here */
	struct block_fota_event *block_ev = new_block_fota_event();
	block_ev->block_lte_fota = false;
	EVENT_SUBMIT(block_ev);
}

static void dfu_pending_cb(void)
{
	LOG_INF("BLE DFU pending");
	/* Reset of the collar will automatically be performed after this step. 
	 * From NCS v2.0.0 this can be controlled manually */
}

img_mgmt_dfu_callbacks_t dfu_callbacks = {
	/* Called when upgrade starts. If upload is paused and restarted this is not called again */
	.dfu_started_cb = &dfu_started_cb,
	/* Called whenever img_mgmt_dfu_stopped() is called, 
	 * which is when something goes wrong with image upload or erase process */
	.dfu_stopped_cb = &dfu_stopped_cb,
	/* Called when upload is compleate and reboot will take place */
	.dfu_pending_cb = &dfu_pending_cb,
	/* Is called when new firmware is confirmed during test boot */
	.dfu_confirmed_cb = NULL
};

static int software_update_confirmation_handler(uint32_t offset, uint32_t size,
						void *arg)
{
	/* For now just print update progress and confirm data chunk without any additional
	 * checks.
	 */
	LOG_INF("Device firmware upgrade progress %d / %d bytes (%d%%)", offset,
		size, ((offset * 100) / size));

	return 0;
}

int bt_dfu_init(void)
{
	int err = 0;
	os_mgmt_register_group();
	img_mgmt_register_group();
	img_mgmt_register_callbacks(&dfu_callbacks);
	img_mgmt_set_upload_cb(software_update_confirmation_handler, NULL);

	err = smp_bt_register();
	if (err) {
		LOG_ERR("SMP BT register failed (err: %d)", err);
	}

	return err;
}