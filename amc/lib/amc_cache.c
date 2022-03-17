/*
 * Copyright (c) 2022 Nofence AS
 */

#include <zephyr.h>
#include <logging/log.h>

#include "pasture_structure.h"
#include "amc_cache.h"
#include "nf_common.h"

LOG_MODULE_REGISTER(amc_cache, CONFIG_AMC_LIB_LOG_LEVEL);

K_SEM_DEFINE(fence_data_sem, 1, 1);
K_SEM_DEFINE(gnss_data_sem, 1, 1);

/** Use two memory regions so we can swap the pointer between them
 * so that instead of waiting for semaphore to be released, we schedule
 * a pointer swap to the other region once we've read the data. Use an
 * atomic variable if both threads try to access and update that
 * gnss data is available or consumed.
 * 
 * @note Need to include correct GNSS header.
 */
static gnss_struct_t cached_gnssdata_area_1;
static gnss_struct_t cached_gnssdata_area_2;
static gnss_struct_t *current_gnssdata_area = &cached_gnssdata_area_1;
atomic_t new_gnss_written = ATOMIC_INIT(false);

/* Cached fence data header and a max coordinate region. Links the coordinate
 * region to the header. Important! Is set to NULL everytime it's free'd
 * to ensure that we're calculating with a valid cached fence.
 */
static pasture_t pasture_cache;

int get_pasture_cache(pasture_t *pasture)
{
	if (pasture_cache.m.ul_total_fences == 0) {
		return -ENODATA;
	}

	pasture = &pasture_cache;
	return 0;
}

int set_pasture_cache(uint8_t *pasture, size_t len)
{
	if (len != sizeof(pasture_t)) {
		return -EINVAL;
	}
	int err = k_sem_take(&fence_data_sem,
			     K_SECONDS(CONFIG_FENCE_CACHE_TIMEOUT_SEC));
	if (err) {
		LOG_ERR("Error semaphore for fence cache %i", err);
		return err;
	}

	/* Clear any previous pasture cache. */
	memset(&pasture_cache, 0, sizeof(pasture_t));

	/* Memcpy the contents. */
	memcpy(&pasture_cache, pasture, sizeof(pasture_t));

	k_sem_give(&fence_data_sem);
	return 0;
}

int set_gnss_cache(gnss_struct_t *gnss)
{
	/* We only need to take semaphore if AMC have not yet consumed
         * the previous GNSS data, which means we have to wait until it
         * has consumed it (swapped READ gnss cache slot). We need it to
         * be available immediately since it's called from the event bus.
         * If it's not available due to AMC still swapping GNSS cache,
         * return that it still hasn't been consumed.
         */
	int err = k_sem_take(&gnss_data_sem, K_NO_WAIT);
	if (err) {
		LOG_ERR("Consumer still swapping previous GNSS data. %i", err);
		return err;
	}

	if (current_gnssdata_area == &cached_gnssdata_area_1) {
		memcpy(&cached_gnssdata_area_2, &gnss, sizeof(gnss_struct_t));
	} else {
		memcpy(&cached_gnssdata_area_1, &gnss, sizeof(gnss_struct_t));
	}

	atomic_set(&new_gnss_written, true);
	k_sem_give(&gnss_data_sem);
	return 0;
}

int get_gnss_cache(gnss_struct_t *gnss)
{
	int err = k_sem_take(&gnss_data_sem,
			     K_SECONDS(CONFIG_GNSS_CACHE_TIMEOUT_SEC));
	if (err) {
		LOG_ERR("Error semaphore for gnss cache %i", err);
		return err;
	}

	/* Fetch GNSS data. Checks if we received new data, 
         * swap the pointer to point at the newly written GNSS data 
         * area if we have. If not, just return the current location.
	 */
	if (atomic_get(&new_gnss_written)) {
		if (current_gnssdata_area == &cached_gnssdata_area_1) {
			current_gnssdata_area = &cached_gnssdata_area_2;
		} else {
			current_gnssdata_area = &cached_gnssdata_area_1;
		}
		atomic_set(&new_gnss_written, false);
	}

	gnss = current_gnssdata_area;

	if (gnss == NULL) {
		return -ENODATA;
	}

	k_sem_give(&gnss_data_sem);
	return 0;
}