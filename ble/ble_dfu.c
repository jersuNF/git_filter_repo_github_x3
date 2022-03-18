/*
 * Copyright (c) 2021 Nofence AS
 */

#include <init.h>

#include <img_mgmt/img_mgmt.h>
#include <os_mgmt/os_mgmt.h>
#include <mgmt/mcumgr/smp_bt.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(ble_dfu);

int bt_dfu_init(void)
{
	int err = 0;

	img_mgmt_register_group();
	os_mgmt_register_group();

	err = smp_bt_register();

	if (err) {
		LOG_ERR("SMP BT register failed (err: %d)", err);
	}

	return err;
}