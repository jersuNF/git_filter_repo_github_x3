/*
 * Copyright (c) 2021 Nofence AS
 */

#include <init.h>

#include <img_mgmt/img_mgmt.h>
#include <os_mgmt/os_mgmt.h>
#include <mgmt/mcumgr/smp_bt.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(ble_dfu);

/**
 * @brief Initialize the SMP service to do BLE FOTA 
 * 
 * @param[in] dev Runtime device. Attribute unused
 * @return 0 on success, negative error code on faliure
 */
static int bt_dfu_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	int err = 0;

	img_mgmt_register_group();
	os_mgmt_register_group();

	err = smp_bt_register();

	if (err) {
		LOG_ERR("SMP BT register failed (err: %d)", err);
	}

	return err;
}

/* This will run bt_dfu_init at system boot */
SYS_INIT(bt_dfu_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
