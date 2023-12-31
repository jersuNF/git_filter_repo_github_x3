#include "nclogs.h"
/*
 * Copyright (c) 2022 Nofence AS
 */

#include <zephyr.h>
#include <logging/log.h>

#include "pasture_structure.h"
#include "amc_cache.h"
#include "amc_states_cache.h"
#include "embedded.pb.h"

LOG_MODULE_REGISTER(amc_cache, CONFIG_AMC_LIB_LOG_LEVEL);
/* Required by generate_nclogs.py*/
#define NCID AMC_MODULE

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
static gnss_t cached_gnssdata_area_1;
static gnss_t cached_gnssdata_area_2;
static gnss_t *current_gnssdata_area = &cached_gnssdata_area_1;
atomic_t new_gnss_written = ATOMIC_INIT(false);
static bool m_gnss_timeout = false;

/* Cached fence data header and a max coordinate region. Links the coordinate
 * region to the header. Important! Is set to NULL everytime it's free'd
 * to ensure that we're calculating with a valid cached fence.
 */
static pasture_t pasture_cache;

int get_pasture_cache(pasture_t **pasture)
{
	if (pasture_cache.m.ul_total_fences == 0) {
		NCLOG_WRN(NCID, TRice0( iD( 7076),"wrn: Switching to \'No pasture\'! \n"));
	}

	*pasture = &pasture_cache;
	return 0;
}

int set_pasture_cache(uint8_t *pasture, size_t len)
{
	if (len != sizeof(pasture_t)) {
		return -EINVAL;
	}
	int err = k_sem_take(&fence_data_sem, K_SECONDS(CONFIG_FENCE_CACHE_TIMEOUT_SEC));
	if (err) {
		NCLOG_ERR(NCID, TRice( iD( 3132),"err: Error semaphore for fence cache %i \n", err));
		return err;
	}

	/* Clear any previous pasture cache. */
	memset(&pasture_cache, 0, sizeof(pasture_t));

	/* Memcpy the contents. */
	memcpy(&pasture_cache, (pasture_t *)pasture, sizeof(pasture_t));

	k_sem_give(&fence_data_sem);
	return 0;
}

int set_gnss_cache(gnss_t *gnss, const bool timed_out)
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
		NCLOG_ERR(NCID, TRice( iD( 7797),"err: Consumer still swapping previous GNSS data. %i \n", err));
		return err;
	}

	m_gnss_timeout = timed_out;
	if (m_gnss_timeout == false) {
		/* GNSS data is only updated if GNSS data is valid, i.e. the 
		 * GNSS has NOT timed out (See gnss_controller) */
		if (current_gnssdata_area == &cached_gnssdata_area_1) {
			memcpy(&cached_gnssdata_area_2, gnss, sizeof(gnss_t));
		} else {
			memcpy(&cached_gnssdata_area_1, gnss, sizeof(gnss_t));
		}

		atomic_set(&new_gnss_written, true);
	}
	k_sem_give(&gnss_data_sem);
	return 0;
}

int get_gnss_cache(gnss_t **gnss)
{
	int err = 0;
	err = k_sem_take(&gnss_data_sem, K_SECONDS(CONFIG_GNSS_CACHE_TIMEOUT_SEC));
	if (err) {
		NCLOG_ERR(NCID, TRice( iD( 6797),"err: Error semaphore for gnss cache %i \n", err));
		return err;
	}

	/* Fetch GNSS data. Checks if we received new valid data, 
         * swap the pointer to point at the newly written GNSS data 
         * area if we have. If not, just return the current location.
	 */
	if ((m_gnss_timeout == false) && (atomic_get(&new_gnss_written) == true)) {
		if (current_gnssdata_area == &cached_gnssdata_area_1) {
			current_gnssdata_area = &cached_gnssdata_area_2;
		} else {
			current_gnssdata_area = &cached_gnssdata_area_1;
		}
		atomic_set(&new_gnss_written, false);
	}

	*gnss = current_gnssdata_area;
	if (gnss == NULL) {
		err = -ENODATA;
	} else if (m_gnss_timeout == true) {
		err = -ETIMEDOUT;
	}
	k_sem_give(&gnss_data_sem);
	return err;
}

bool fnc_valid(fence_t *fence)
{
	return (fence->m.n_points > 2 && fence->m.n_points <= 40 &&
		(fence->m.e_fence_type == FenceDefinitionMessage_FenceType_Normal ||
		 fence->m.e_fence_type == FenceDefinitionMessage_FenceType_Inverted));
	/** @todo: [LEGACY CODE] Also test timespan from storage fence definition here. */
}

bool fnc_valid_fence(void)
{
	pasture_t *pasture = NULL;

	if (get_pasture_cache(&pasture)) {
		return false;
	}

	if ((get_fence_status() == FenceStatus_FenceStatus_Invalid) ||
	    (get_fence_status() == FenceStatus_TurnedOffByBLE)) {
		return false;
	}

	for (int i = 0; i < pasture->m.ul_total_fences; i++) {
		if (!fnc_valid(&pasture->fences[i])) {
			return false;
		}
	}
	return pasture->m.ul_total_fences != 0;
}

bool fnc_valid_def(void)
{
	pasture_t *pasture = NULL;

	if (get_pasture_cache(&pasture)) {
		return false;
	}
	return pasture->m.ul_fence_def_version != 0;
}
