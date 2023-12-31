/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _STORAGE_H_
#define _STORAGE_H_

#include <zephyr.h>

#if defined(CONFIG_MESSAGING) &&                                                                   \
	defined(CONFIG_ZTEST) /* TODO: decouple the storage_controller code from messaging for cleaner/easier testing */
typedef int (*fcb_read_cb)(uint8_t *data, size_t len);
#else
#include "fcb_ext.h"
#endif

#include "ano_structure.h"
#include "storage_event.h"

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
 * @note Padding is added if the payload is not a multiple of 4. The caller 
 *       has to handle the removal of the padding itself, and also include
 *       the length in the first two bytes of the incomming data packet.
 * 
 * @param[in] partition which partition to write to.
 * @param[in] data input where the flash data is written from.
 * @param[in] len input regarding the size of the written data.
 * @param[in] rotate_to_this Clears all the previous entries if this is true
 *                           making the current entry the only one present.
 * 
 * @return 0 on success, otherwise negative errno.
 */
int stg_write_to_partition(flash_partition_t partition, uint8_t *data, size_t len);

/** 
 * @brief Reads all the new available seq data and calls the callback function
 *        with all the unread seq entries.
 * 
 * @param[in] cb pointer location to the callback function that is 
 *               called during the fcb walk.
 * @param[in] num_entries number of entries we want to read. If 0, read all.
 * 
 * @return 0 on success 
 * @return -ENODATA if no data available, Otherwise negative errno.
 */
int stg_read_seq_data(fcb_read_cb cb, uint16_t num_entries);

/** 
 * @brief Reads newest ano data and calls cb for each entry.
 * 
 * @param[in] cb pointer location to the callback function that is 
 *               called during the fcb walk.
 * @param[in] cb_arg passed to the callback function as the cb_arg parameter
 * @param[in] read_from_start If true, read from oldest FIFO entry, if true, read from last retrieved
 * location
 * @param[in] num_entries number of entries we want to read. If 0, read all.
 * 
 * @return 0 on success 
 * @return -ENODATA if no data available, Otherwise negative errno.
 */
int stg_read_ano_data(fcb_read_cb cb, bool read_from_start, uint16_t num_entries);

/** 
 * @brief Reads the newest pasture and callbacks the data.
 * 
 * @param[in] cb pointer location to the callback function that is 
 *               called during the data read.
 * 
 * @return 0 on success 
 * @return -ENODATA if no data available, Otherwise negative errno.
 */
int stg_read_pasture_data(fcb_read_cb cb);

/** 
 * @brief Writes seq data to external flash SEQ partition.
 * 
 * @param[in] data pointer location to of data to be written
 * @param[in] len length of data
 * 
 * @return 0 on success, otherwise negative errno
 */
int stg_write_seq_data(uint8_t *data, size_t len);

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
int stg_write_ano_data(const ano_rec_t *ano_rec);

/** 
 * @brief Writes pasture data to external flash PASTURE partition.
 * 
 * @param[in] data pointer location to of data to be written
 * @param[in] len length of data
 * 
 * @return 0 on success, otherwise negative errno
 */
int stg_write_pasture_data(uint8_t *data, size_t len);

/** 
 * @brief Checks if seq active pointer is pointing to last element, return
 *        true is this is the case.
 * 
 * @return 0(false) if not empty, 1(true) if empty (pointing to last).
 */
bool stg_seq_pointing_to_last();

/** 
 * @brief Returns the amount of entries on the given partition.
 * 
 * @param[in] partition which partition to check entries from.
 * 
 * @return number of entries on partition.
 */
uint32_t get_num_entries(flash_partition_t partition);

/** 
 * @brief Reads the newest system diagnostic log and callbacks the data.
 * 
 * @param[in] cb pointer location to the callback function that is 
 *               called during the data read.
 * 
 * @return 0 on success 
 * @return -ENODATA if no data available, Otherwise negative errno.
 */
int stg_read_system_diagnostic_log(fcb_read_cb cb, uint16_t num_entries);

/** 
 * @brief Writes log data to external flash SYSTEMDIAG partition.
 * 
 * @param[in] data pointer location to of data to be written
 * @param[in] len length of data
 * 
 * @return 0 on success, otherwise negative errno
 */
int stg_write_system_diagnostic_log(uint8_t *data, size_t len);

#define SECTOR_SIZE MAX(CONFIG_NORDIC_QSPI_NOR_FLASH_LAYOUT_PAGE_SIZE, CONFIG_STORAGE_SECTOR_SIZE)

#define FLASH_SEQ_NUM_SECTORS PM_SEQ_PARTITION_SIZE / SECTOR_SIZE

#define FLASH_ANO_NUM_SECTORS PM_ANO_PARTITION_SIZE / SECTOR_SIZE

#define FLASH_PASTURE_NUM_SECTORS PM_PASTURE_PARTITION_SIZE / SECTOR_SIZE

#define FLASH_SYSTEM_DIAG_NUM_SECTORS PM_SYSTEM_DIAGNOSTIC_SIZE / SECTOR_SIZE

#if defined(CONFIG_MESSAGING) && defined(CONFIG_ZTEST)
#define CONFIG_CORE_DUMP_FLASH_MAGIC_FLAG 0xA1A10000
typedef enum partition {
	coredump_partition = 0,
} partition_name;

uint32_t FLASH_AREA_OFFSET(partition_name name);
void simulate_valid_coredump(size_t total_words);
int write_to_core_dump_partition(size_t index, uint32_t val);
bool is_valid_core_dump();
void nrfx_nvmc_words_write(size_t address, uint32_t *ptr_word, size_t number_of_words);
#endif

#endif /* _STORAGE_H_ */
