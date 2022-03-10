/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _STORAGE_H_
#define _STORAGE_H_

#include <zephyr.h>
#include "fcb_ext.h"

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
 * @param[in] rotate_to_this Clears all the previous entries if this is true
 *                           making the current entry the only one present.
 * 
 * @return 0 on success, otherwise negative errno.
 */
int stg_write_to_partition(flash_partition_t partition, uint8_t *data,
			   size_t len);

/** 
 * @brief Reads all the new available log data and calls the callback function
 *        with all the unread log entries.
 *        The caller must deal with allocation itself.
 * 
 * @param[in] cb pointer location to the callback function that is 
 *               called during the fcb walk.
 * @param[in] num_entries number of entries we want to read. If 0, read all.
 * 
 * @return 0 on success 
 * @return -ENODATA if no data available, Otherwise negative errno.
 */
int stg_read_log_data(fcb_read_cb cb, uint16_t num_entries);

/** 
 * @brief Reads newest ano data and stores the data onto 
 *        the pointer location. The caller must deal with allocation itself.
 * 
 * @param[in] cb pointer location to the callback function that is 
 *               called during the fcb walk.
 * @param[in] last_valid_ano  If false, reads from last known sent ANO frame.
 *                            If true, reads from active_ano_entry which
 *                            points to oldest entry with valid ANO data.
 *                            This pointer is updated using update_ano_active_entry.
 * @param[in] num_entries number of entries we want to read. If 0, read all.
 * 
 * @return 0 on success 
 * @return -ENODATA if no data available, Otherwise negative errno.
 */
int stg_read_ano_data(fcb_read_cb cb, bool last_valid_ano,
		      uint16_t num_entries);

/** 
 * @brief Reads the newest pasture and stores the data onto 
 *        the pointer location. The caller must deal with allocation itself.
 * 
 * @param[in] cb pointer location to the callback function that is 
 *               called during the fcb walk.
 * 
 * @return 0 on success 
 * @return -ENODATA if no data available, Otherwise negative errno.
 */
int stg_read_pasture_data(fcb_read_cb cb);

/** 
 * @brief Writes log data to external flash LOG partition.
 * 
 * @param[in] data pointer location to of data to be written
 * @param[in] len length of data
 * 
 * @return 0 on success, otherwise negative errno
 */
int stg_write_log_data(uint8_t *data, size_t len);

/** 
 * @brief Writes ano data to external flash ANO partition.
 * 
 * @param[in] data pointer location to of data to be written
 * @param[in] len length of data
 * @param[in] first_frame used to intialize the ANO partitions if 
 *                        its the frist frame
 * 
 * @return 0 on success, otherwise negative errno
 */
int stg_write_ano_data(uint8_t *data, size_t len);

/** 
 * @brief Writes log data to external flash LOG partition.
 * 
 * @param[in] data pointer location to of data to be written
 * @param[in] len length of data
 * 
 * @return 0 on success, otherwise negative errno
 */
int stg_write_pasture_data(uint8_t *data, size_t len);

#define SECTOR_SIZE                                                            \
	MAX(CONFIG_NORDIC_QSPI_NOR_FLASH_LAYOUT_PAGE_SIZE,                     \
	    CONFIG_STORAGE_SECTOR_SIZE)

#define FLASH_LOG_NUM_SECTORS PM_LOG_PARTITION_SIZE / SECTOR_SIZE

#define FLASH_ANO_NUM_SECTORS PM_ANO_PARTITION_SIZE / SECTOR_SIZE

#define FLASH_PASTURE_NUM_SECTORS PM_PASTURE_PARTITION_SIZE / SECTOR_SIZE

#endif /* _STORAGE_H_ */