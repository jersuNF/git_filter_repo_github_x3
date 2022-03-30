/*
 * Copyright (c) 2021 Nofence AS
 */
#include "amc_cache.h"
#include "amc_dist.h"
#include "amc_zone.h"
#include "amc_gnss.h"
#include "amc_states.h"
#include "amc_correction.h"
#include "amc_const.h"
#include "nf_fifo.h"

#include <zephyr.h>
#include "amc_handler.h"
#include <logging/log.h>
#include "sound_event.h"
#include "request_events.h"
#include "pasture_structure.h"
#include "event_manager.h"
#include "error_event.h"
#include "messaging_module_events.h"
#include "gnss_controller_events.h"

#include "ble_beacon_event.h"

#include "storage.h"

/* Distance arrays. */
static int16_t dist_array[FIFO_ELEMENTS];
static int16_t dist_avg_array[FIFO_AVG_DISTANCE_ELEMENTS];

/* Array to discover unstable position accuracy. */
static int16_t acc_array[FIFO_ELEMENTS];

/* Array used to discover abnormal height changes that might happen
 * if gps signals gets reflected.
 */
static int16_t height_avg_array[FIFO_ELEMENTS];

/* Static variables used in AMC logic. */
static int16_t dist_change;
static int16_t mean_dist = INT16_MIN;
static int16_t instant_dist = INT16_MIN;
static uint8_t fifo_dist_elem_count = 0;
static uint8_t fifo_avg_dist_elem_count = 0;

#define MODULE animal_monitor_control
LOG_MODULE_REGISTER(MODULE, CONFIG_AMC_LOG_LEVEL);

/* Thread stack area that we use for the calculation process. We add a work
 * item here when we have data available from GNSS.
 * This thread can then use the calculation function algorithm to
 * determine what it needs to do in regards to sound and zap events that
 * this calculation function can submit to event handler.
 */
K_THREAD_STACK_DEFINE(amc_calculation_thread_area, CONFIG_AMC_CALCULATION_SIZE);

static struct k_work_q amc_work_q;
static struct k_work process_new_gnss_work;
static struct k_work process_new_fence_work;

static inline int update_pasture_from_stg(void)
{
	int err = stg_read_pasture_data(set_pasture_cache);
	if (err == -ENODATA) {
		char *err_msg = "No pasture found on external flash.";
		nf_app_warning(ERR_AMC, err, err_msg, strlen(err_msg));
		return 0;
	} else if (err) {
		char *err_msg = "Couldn't update pasture cache in AMC.";
		nf_app_fatal(ERR_AMC, err, err_msg, strlen(err_msg));
		return err;
	}
	return 0;
}

void process_new_fence_fn(struct k_work *item)
{
	/* Update AMC cache. */
	int cache_ret = update_pasture_from_stg();

	/* Take pasture sem, since we need to access the version to send
	 * to messaging module.
	 */
	int err = k_sem_take(&fence_data_sem,
			     K_SECONDS(CONFIG_FENCE_CACHE_TIMEOUT_SEC));
	if (err) {
		LOG_ERR("Error taking pasture semaphore for version check.");
		return;
	}

	pasture_t *pasture = NULL;
	get_pasture_cache(&pasture);

	if (cache_ret) {
		pasture->m.ul_fence_def_version = 0;
		LOG_ERR("Error caching new pasture from storage controller.");
		return;
	}

	/* Submit event that we have now began to use the new fence. */
	struct update_fence_version *ver = new_update_fence_version();
	ver->fence_version = pasture->m.ul_fence_def_version;
	EVENT_SUBMIT(ver);

	k_sem_give(&fence_data_sem);
}

/**
 * @brief Work function for processing new gnss data that have just been stored.
 */
void process_new_gnss_data_fn(struct k_work *item)
{
	/* Take fence semaphore since we're going to use the cached area. */
	int err = k_sem_take(&fence_data_sem,
			     K_SECONDS(CONFIG_FENCE_CACHE_TIMEOUT_SEC));
	if (err) {
		LOG_ERR("Error waiting for fence data semaphore to release.");
		goto cleanup;
	}

	/* Fetch cached fence and size. */
	pasture_t *pasture = NULL;

	err = get_pasture_cache(&pasture);
	if (err || pasture == NULL) {
		/* Error handle. */
		goto cleanup;
	}

	/* Fetch new, cached gnss data. */
	gnss_t *gnss = NULL;
	err = get_gnss_cache(&gnss);
	if (err || gnss == NULL) {
		/* Error handle. */
		goto cleanup;
	}

	/* Set local variables used in AMC logic. */
	int16_t height_delta = INT16_MAX;
	int16_t acc_delta = INT16_MAX;
	int16_t dist_avg_change = 0;
	uint8_t dist_inc_count = 0;

	/* Update static variables pre-fix check. */
	amc_zone_t cur_zone = zone_get();

	/* Validate position */
	err = gnss_update(gnss);
	if (err) {
		/* Error handle. */
		goto cleanup;
	}

	/* Fetch x and y position based on gnss data */
	int16_t pos_x = 0, pos_y = 0;
	err = gnss_calc_xy(gnss, &pos_x, &pos_y, pasture->m.l_origin_lon,
			   pasture->m.l_origin_lat, pasture->m.us_k_lon,
			   pasture->m.us_k_lat);
	bool overflow_xy = err == -EOVERFLOW;

	/* If any fence (pasture?) is valid and we have fix. */
	if (gnss_has_fix() && fnc_any_valid_fence() && !overflow_xy) {
		/* Calculate distance to closest polygon. */
		uint8_t fence_index = 0;
		uint8_t vertex_index = 0;
		instant_dist = fnc_calc_dist(pos_x, pos_y, &fence_index,
					     &vertex_index);

		/* Reset dist_change since we acquired a new distance. */
		dist_change = 0;

		/* if ((GPS_GetMode() == GPSMODE_MAX) 
		*  && TestBits(m_u16_PosAccuracy, GPSFIXOK_MASK)) { ??????
	 	*/
		/** @todo We should do something about that GPS_GetMode thing 
		  * here. Either add check to has_accepted_fix, or explicit 
		  * call to gnss_get_mode */
		if (gnss_has_accepted_fix()) {
			/* Accepted position. Fill FIFOs. */
			fifo_put(gnss->lastfix.h_acc_dm, acc_array,
				 FIFO_ELEMENTS);
			fifo_put(gnss->lastfix.height, height_avg_array,
				 FIFO_ELEMENTS);
			fifo_put(instant_dist, dist_array, FIFO_ELEMENTS);

			/* If we have filled the distance FIFO, calculate
			 * the average and store that value into
			 * another avg_dist FIFO.
			 */
			if (++fifo_dist_elem_count >= FIFO_ELEMENTS) {
				fifo_dist_elem_count = 0;
				fifo_put(fifo_avg(dist_array, FIFO_ELEMENTS),
					 dist_avg_array,
					 FIFO_AVG_DISTANCE_ELEMENTS);

				if (++fifo_avg_dist_elem_count >=
				    FIFO_AVG_DISTANCE_ELEMENTS) {
					fifo_avg_dist_elem_count =
						FIFO_AVG_DISTANCE_ELEMENTS;
				}
			}
		} else {
			fifo_dist_elem_count = 0;
			fifo_avg_dist_elem_count = 0;
		}

		if (fifo_avg_dist_elem_count > 0) {
			/* Fill avg/mean/delta fifos as we have collected
			 * valid data over a short period.
			 */
			mean_dist = fifo_avg(dist_array, FIFO_ELEMENTS);
			dist_change = fifo_slope(dist_array, FIFO_ELEMENTS);
			dist_inc_count =
				fifo_inc_cnt(dist_array, FIFO_ELEMENTS);

			acc_delta = fifo_delta(acc_array, FIFO_ELEMENTS);
			height_delta =
				fifo_delta(height_avg_array, FIFO_ELEMENTS);

			if (fifo_avg_dist_elem_count >=
			    FIFO_AVG_DISTANCE_ELEMENTS) {
				dist_avg_change =
					fifo_slope(dist_avg_array,
						   FIFO_AVG_DISTANCE_ELEMENTS);
			}
		}

		int16_t dist_incr_slope_lim = 0;
		uint8_t dist_incr_count = 0;

		/* Set slopes and count based on mode. */
		if (get_mode() == Mode_Teach) {
			dist_incr_slope_lim = TEACHMODE_DIST_INCR_SLOPE_LIM;
			dist_incr_count = TEACHMODE_DIST_INCR_COUNT;
		} else {
			dist_incr_slope_lim = DIST_INCR_SLOPE_LIM;
			dist_incr_count = DIST_INCR_COUNT;
		}

		/* Set final accuracy flags based on previous calculations. */
		err = gnss_update_dist_flags(dist_avg_change, dist_change,
					     dist_incr_slope_lim,
					     dist_inc_count, dist_incr_count,
					     height_delta, acc_delta, mean_dist,
					     gnss->lastfix.h_acc_dm);
		if (err) {
			/* Error handle. */
			goto cleanup;
		}

		/* Update zone. */
		err = zone_update(instant_dist, gnss, &cur_zone);
		if (err != 0) {
			fifo_dist_elem_count = 0;
			fifo_avg_dist_elem_count = 0;
		}

		/* Start correction and set states based on fence status.
		 * Should we have this is another loop, or just perform
		 * whenever we get new postion data?
		 */
		Mode amc_mode = calc_mode();
		FenceStatus fence_status = calc_fence_status(k_uptime_get_32());

		/** @todo CollarStatus collar_status = get_collar_status();
		 */

		/** @todo 
		 * 
		 * Check for gnss->haslastfix?
		 * gnss_mode_t gnss_mode = gnss->lastfix.mode;
		 * err = set_sensor_modes(amc_mode, gnss_mode, fence_status,
		 * 		       collar_status);
		 * if (err) {
		 * 	goto cleanup;
		 * }
		 */

		/** @todo GNSS set mode HERE! 
		 *
		 * err = set_gnss_mode(amc_mode, fence_status, collar_status);
		 * if (err) {
		 * 	goto cleanup;
		 * }
		 */

		err = process_correction(amc_mode, gnss->lastfix, fence_status,
					 cur_zone, mean_dist, dist_change);
		if (err) {
			/* Error handle. */
			goto cleanup;
		}
	} else {
		fifo_dist_elem_count = 0;
		fifo_avg_dist_elem_count = 0;
		zone_set(NO_ZONE);
	}

cleanup:
	/* Calculation finished, give semaphore so we can swap memory region
	 * on next GNSS request. 
	 * As well as notifying we're not using fence data area. 
	 */
	k_sem_give(&fence_data_sem);
}

int amc_module_init(void)
{
	/* Init work item and start and init calculation 
	 * work queue thread and item. 
	 */
	k_work_queue_init(&amc_work_q);
	k_work_queue_start(&amc_work_q, amc_calculation_thread_area,
			   K_THREAD_STACK_SIZEOF(amc_calculation_thread_area),
			   CONFIG_AMC_CALCULATION_PRIORITY, NULL);
	k_work_init(&process_new_gnss_work, process_new_gnss_data_fn);
	k_work_init(&process_new_fence_work, process_new_fence_fn);

	/* Checks and inits the mode we're in. */
	init_mode_status();

	/* Fetch the fence from external flash and update fence cache. */
	return update_pasture_from_stg();
}

/**
 * @brief Main event handler function. 
 * 
 * @param[in] eh Event_header for the if-chain to 
 *               use to recognize which event triggered.
 * 
 * @return True or false based on if we want to consume the event or not.
 */
static bool event_handler(const struct event_header *eh)
{
	if (is_new_fence_available(eh)) {
		k_work_submit_to_queue(&amc_work_q, &process_new_fence_work);
		return false;
	}
	if (is_gnss_data(eh)) {
		struct gnss_data *event = cast_gnss_data(eh);

		int err = set_gnss_cache(&event->gnss_data);
		if (err) {
			char *msg = "Could not set gnss cahce.";
			nf_app_error(ERR_AMC, err, msg, strlen(msg));
			return false;
		}

		k_work_submit_to_queue(&amc_work_q, &process_new_gnss_work);
		return false;
	}
	if (is_ble_beacon_event(eh)) {
		struct ble_beacon_event *event = cast_ble_beacon_event(eh);

		set_beacon_status(event->status);
		return false;
	}
	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, gnss_data);
EVENT_SUBSCRIBE(MODULE, new_fence_available);
EVENT_SUBSCRIBE(MODULE, ble_beacon_event);
