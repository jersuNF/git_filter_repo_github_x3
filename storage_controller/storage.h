/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _STORAGE_H_
#define _STORAGE_H_

#include <zephyr.h>
#include "storage_events.h"
/** 
 * @brief Setup external flash driver
 * 
 * @return 0 if successful, otherwise a negative error code.
 */
int init_storage_controller(void);

/** 
 * @brief Setup external flash driver
 * @param partition which partition to clear fcb
 * @return 0 if successful, otherwise a negative error code.
 */
int clear_fcb_sectors(flash_partition_t partition);

/** @brief Helper function to reset the FCBs, used to simulate persistent
 *         storage. Used by unit tests and could also be used by other
 *         functions/APIs to reset the FCBs. Also calls init_storage_controller.
 * @return 0 on success, otherwise negative errno.
 */
int stg_fcb_reset_and_init();

#endif /* _STORAGE_H_ */