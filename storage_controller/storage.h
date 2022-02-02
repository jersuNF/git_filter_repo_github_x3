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

#define SECTOR_SIZE                                                            \
	MAX(CONFIG_NORDIC_QSPI_NOR_FLASH_LAYOUT_PAGE_SIZE,                     \
	    CONFIG_STORAGE_SECTOR_SIZE)

#define FLASH_LOG_NUM_SECTORS PM_LOG_PARTITION_SIZE / SECTOR_SIZE
#define FLASH_ANO_NUM_SECTORS PM_ANO_PARTITION_SIZE / SECTOR_SIZE
#define FLASH_PASTURE_NUM_SECTORS PM_PASTURE_PARTITION_SIZE / SECTOR_SIZE

#endif /* _STORAGE_H_ */