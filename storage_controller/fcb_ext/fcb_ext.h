/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _FCB_EXT_H_
#define _FCB_EXT_H_

#include <zephyr.h>
#include <fs/fcb.h>

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
typedef int (*fcb_read_cb)(uint8_t *data, size_t len);

/** 
 * @brief Starts to read from start_entry to fcb.f_active's last entry.
 *        Callbacks each entry which gives the entry struct, raw read data and
 *        the data length.
 * 
 * @param[in] cb callback structure for each entry retrieved.
 * @param[in] fcb pointer to fcb.
 * @param[inout] start_entry entry to start to read from. Is updated regularly
 *                           if caller wants to know which entry is being read.
 *                           Useful when reading stops mid-read, meaning
 *                           we can continue from where we left off.
 * @param num_entries number of entries we want to callback. Set to 0 to read
 *                    all entries. The function will exit regardless once all
 *                    entries are consumed if this number 
 *                    is higher than entries that exists.
 * 
 * @return 0 on success, otherwise negative errno.
 * @return -EINTR if caller aborted walk process.
 */
int fcb_walk_from_entry(fcb_read_cb cb, struct fcb *fcb, struct fcb_entry *start_entry,
			uint16_t num_entries, struct k_mutex *flash_mutex);

#endif /* _FCB_EXT_H_ */