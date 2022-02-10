/*
 * Copyright (c) 2021 Nofence AS
 */

#ifndef _FW_UPGRADE_H_
#define _FW_UPGRADE_H_

#include <zephyr.h>
#include "event_manager.h"

/**
 * @brief Used to initilize the firmware upgrade module. 
 *        This calls the DFU status event
 *        and sets it to IDLE and no upgrade in progress, 
 *        as well as cleanup DFU library and set 
 *        bytes written to 0.
 * 
 * @return 0 on success, otherwise negative errno.
 */
int fw_upgrade_module_init();

/**
 * @brief Function that marks the currently running image as valid.
 *        Should be called once we know that the system operates as expected.
 */
void mark_new_application_as_valid();

#endif /* _FW_UPGRADE_H_ */