/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _AMC_CACHE_H_

#include <zephyr.h>
#include "pasture_structure.h"
#include "nf_common.h"

extern struct k_sem fence_data_sem;

/**
 * @brief Fetches the cached fence data and outputs the length and 
 *        pointer location to the cached fence.
 * 
 * @note Semaphore locking when using the cached area must be handled outside
 *       this function, since we give the caller a pointer to the area.
 * 
 * @param[out] fence pointer to where the cached fence is stored.
 * @param[out] len length of the fence struct.
 * 
 * @return 0 on success.
 * @return -ENODATA if fence has 0 fence points
 * @return -ENODATA if fence cache is empty
 */
int get_pasture_cache(fence_t *fence, size_t *len);

/**
 * @brief Free's previous fence cache and stores a new one based on
 *        input data. It is of type uint8_t* and not fence_t* since this
 *        function is fed into the storage controller which reads out
 *        raw binary data.
 * 
 * @param[in] fence pointer to where the fence is buffered.
 * @param[in] len length of the fence struct.
 * 
 * @return 0 on success, otherwise negative errno.
 */
int set_pasture_cache(uint8_t *fence, size_t len);

/** @note Need to include correct gnss struct. BUILD ERROR. */

/**
 * @brief Fetches the cached gnss data and outputs the
 *        pointer location to the cached gnss area.
 * 
 * @note No need to lock semaphores outside, since we use two gnss cached
 *       areas. Whenever we consume gnss_cache, we check if we have new
 *       data written to the other cached area, and swap them if its true.
 * 
 * @param[out] gnss pointer to where the cached gnss is stored.
 * 
 * @return 0 on success.
 * @return -ENODATA if gnss cache has not been set.
 */
int get_gnss_cache(gnss_struct_t *gnss);

/**
 * @brief Updates the gnss cache area that is not being read from. Locks
 *        If consumer has not yet processed the previous GNSS entry.
 * 
 * @param[in] gnss gnss data to cache.
 * 
 * @return 0 on success, otherwise negative errno on semaphore error.
 */
int set_gnss_cache(gnss_struct_t *gnss);

#endif /* _AMC_CACHE_H_ */