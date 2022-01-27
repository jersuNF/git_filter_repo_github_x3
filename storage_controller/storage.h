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

#endif /* _STORAGE_H_ */