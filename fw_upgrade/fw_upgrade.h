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
int apply_fragment(const uint8_t *fragment, size_t fragment_size,
		   size_t file_size);

#endif /* _FW_UPGRADE_H_ */