/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _STORAGE_H_
#define _STORAGE_H_

#include <zephyr.h>

/** Used to tell storage controller which region to read/write to. */
typedef enum {
	STG_PARTITION_LOG = 0,
	STG_PARTITION_ANO = 1,
	STG_PARTITION_PASTURE = 2
} flash_partition_t;

/** 
 * @brief Setup external flash driver
 * 
 * @return 0 if successful, otherwise a negative error code.
 */
int stg_init_storage_controller(void);

/** 
 * @brief Clears all of the content on given partition.
 * 
 * @param[in] partition which partition to clear.
 * 
 * @return 0 if successful, otherwise a negative error code.
 */
int stg_clear_partition(flash_partition_t partition);

/** 
 * @brief Helper function to reset the FCBs, used to simulate persistent
 *        storage. Used by unit tests and could also be used by other
 *        functions/APIs to reset the FCBs. Also calls init_storage_controller.
 *        Does not do anything with the contents on the flash.
 * 
 * @return 0 on success, otherwise negative errno.
 */
int stg_fcb_reset_and_init();

/** 
 * @brief Writes to target partition area and stores the data onto 
 *        external flash based on the given pointer. 
 *        The caller must deal with given data allocation itself.
 * 
 * @param[in] partition which partition to write to.
 * @param[in] data input where the flash data is written from.
 * @param[in] len input regarding the size of the written data.
 * 
 * @return 0 on success, otherwise negative errno.
 */
int stg_write_to_partition(flash_partition_t partition, uint8_t *data,
			   size_t len);

/** 
 * @brief Callback function containing the read entry data. Created
 *        by the caller, which also means we do not 
 *        link in a partition since the caller should have a unique
 *        callback function of what they want to do with the data. We just 
 *        give them some data and a length for every entry.
 * 
 * @param[in] data the raw data pointer.
 * @param[in] len size of the data.
 * 
 * @return 0 on success, otherwise negative errno.
 */
typedef int (*stg_read_log_cb)(uint8_t *data, size_t len);

/** 
 * @brief Reads all the new available log data and calls the callback function
 *        with all the unread log entries.
 *        The caller must deal with allocation itself.
 * 
 * @param[in] cb pointer location to the callback function that is 
 *               called during the fcb walk.
 * 
 * @return 0 on success 
 * @return -ENODATA if no data available, Otherwise negative errno.
 */
int stg_read_log_data(stg_read_log_cb cb);

/** 
 * @brief Reads newest ano data and stores the data onto 
 *        the pointer location. The caller must deal with allocation itself.
 * 
 * @param[in] cb pointer location to the callback function that is 
 *               called during the fcb walk.
 * 
 * @return 0 on success 
 * @return -ENODATA if no data available, Otherwise negative errno.
 */
int stg_read_ano_data(stg_read_log_cb cb);

/** 
 * @brief Reads the newest pasture and sttores the data onto 
 *        the pointer location. The caller must deal with allocation itself.
 * 
 * @param[in] cb pointer location to the callback function that is 
 *               called during the fcb walk.
 * 
 * @return 0 on success 
 * @return -ENODATA if no data available, Otherwise negative errno.
 */
int stg_read_pasture_data(stg_read_log_cb cb);

#define SECTOR_SIZE                                                            \
	MAX(CONFIG_NORDIC_QSPI_NOR_FLASH_LAYOUT_PAGE_SIZE,                     \
	    CONFIG_STORAGE_SECTOR_SIZE)

#define FLASH_LOG_NUM_SECTORS PM_LOG_PARTITION_SIZE / SECTOR_SIZE
#define FLASH_ANO_NUM_SECTORS PM_ANO_PARTITION_SIZE / SECTOR_SIZE
#define FLASH_PASTURE_NUM_SECTORS PM_PASTURE_PARTITION_SIZE / SECTOR_SIZE

#endif /* _STORAGE_H_ */