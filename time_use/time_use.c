/*
 * Copyright (c) 2021 Nofence AS
 */
#include <stdio.h>
#include <zephyr.h>
#include <logging/log.h>
#include "gnss_controller_events.h"
#include "cellular_controller_events.h"
#include "collar_protocol.h"
#include "ble_beacon_event.h"
#include "movement_events.h"
#include <power/reboot.h>
#include "amc_events.h"
#include "messaging_module_events.h"
#include "pwr_event.h"
#include "amc_gnss.h"
#include "histogram_events.h"
#include "time_use.h"
#include "time_use_helpers.h"

#define TIME_USE_THREAD_PRIORITY CONFIG_TIME_USE_THREAD_PRIORITY
#define MODULE time_use
LOG_MODULE_REGISTER(MODULE, CONFIG_TIME_USE_LOG_LEVEL);

/**
 * @brief Helper enum to determine legacy animal state
 */
typedef enum { RESTING, WALKING, RUNNING, UNKNOWN } ANIMAL_STATE;

/**
 * @brief NAV-PVT PSM state values, see the u-blox M10 manual
 */
typedef enum {
	DISABLED = 0x00,
	ENABLED = 0x01,
	ACQUSITION = 0x02,
	TRACKING = 0x03,
	POWER_OPTIMIZED_TRACKING = 0x04,
	INACTIVE = 0x05,
} PSMMode_t;

/**
 * @brief container for data which is shared between the event handler thread
 * and the statistics collector thread.
 */
static struct shared_state {
	amc_zone_t cur_zone;
	CollarStatus cur_collar_status;
	FenceStatus cur_fence_status;
	acc_activity_t cur_activity_level;
	gnss_mode_t cur_gnss_pwr_m;
	uint8_t cur_gnss_pvt_flags;
	uint16_t cur_steps;
	int16_t cur_height_max;
	int16_t cur_height_min;
	int32_t cur_height_sum;
	uint32_t cur_gnss_samples;
	uint16_t cur_speed_min;
	uint16_t cur_speed_max;
	uint32_t cur_speed_sum;
	modem_pwr_mode cur_modem_pwr_m;
	int16_t cur_fresh_pos[2];
	_HISTOGRAM_GNSS_MODES cur_gnss_modes;
	_BATTERY_QC qc_battery;
	uint16_t pos_count;
} shared_state = { .cur_activity_level = ACTIVITY_NO,
		   .cur_gnss_pwr_m = GNSSMODE_NOMODE,
		   .cur_height_max = INT16_MIN,
		   .cur_height_min = INT16_MAX,
		   .cur_speed_min = UINT16_MAX,
		   .cur_modem_pwr_m = POWER_ON,
		   .qc_battery = { .usVbattMin = UINT16_MAX } };

static bool save_and_reset;

static K_THREAD_STACK_DEFINE(collect_stats_stack, CONFIG_TIME_USE_THREAD_STACK_SIZE);
static struct k_thread collect_stats_thread;

static void collect_stats_fn(void);

static K_MUTEX_DEFINE(update_in_progress);

K_MSGQ_DEFINE(histogram_msgq, sizeof(struct collar_histogram), 1, 4);

/**
 * Resets the agregated states in the helper structure. Not all fields
 * should be reset, only the ones not needing to maintaining states between
 * the ~250 ms stats calculation
 * @param state
 */
static void reset_state(struct shared_state *state)
{
	state->cur_steps = 0;
	state->cur_height_max = INT16_MIN;
	state->cur_height_min = INT16_MAX;
	state->cur_speed_min = UINT16_MAX;
	state->cur_speed_max = 0;
	state->cur_gnss_samples = 0;
	state->cur_speed_sum = 0;
	state->cur_height_sum = 0;
	state->qc_battery.usVbattMin = UINT16_MAX;
	state->qc_battery.usVbattMax = 0;
	state->pos_count = 0;
	memset(&(state->cur_gnss_modes), 0, sizeof(state->cur_gnss_modes));
}

static uint16_t clamp_int16(int16_t val)
{
	if (val < 0) {
		return 0;
	} else if (val == INT16_MAX) {
		return UINT16_MAX;
	} else {
		return (uint16_t)val;
	}
}

int time_use_module_init(void)
{
	LOG_INF("Initializing the time_use module.");

	k_thread_create(&collect_stats_thread, collect_stats_stack,
			K_THREAD_STACK_SIZEOF(collect_stats_stack),
			(k_thread_entry_t)collect_stats_fn, NULL, NULL, NULL,
			TIME_USE_THREAD_PRIORITY, 0, K_NO_WAIT);
	return 0;
}

static bool event_handler_impl(const struct event_header *eh)
{
	if (is_zone_change(eh)) {
		struct zone_change *ev = cast_zone_change(eh);
		shared_state.cur_zone = ev->zone;
		return false;
	} else if (is_update_collar_status(eh)) {
		struct update_collar_status *ev = cast_update_collar_status(eh);
		shared_state.cur_collar_status = ev->collar_status;
		return false;
	} else if (is_update_fence_status(eh)) {
		struct update_fence_status *ev = cast_update_fence_status(eh);
		shared_state.cur_fence_status = ev->fence_status;
		return false;
	} else if (is_activity_level(eh)) {
		struct activity_level *ev = cast_activity_level(eh);
		shared_state.cur_activity_level = ev->level;
		return false;
	} else if (is_step_counter_event(eh)) {
		struct step_counter_event *ev = cast_step_counter_event(eh);
		shared_state.cur_steps += (uint16_t)ev->steps;
		return false;
	} else if (is_gnss_data(eh)) {
		struct gnss_data *ev = cast_gnss_data(eh);
		shared_state.cur_gnss_pwr_m = ev->gnss_data.latest.mode;
		shared_state.cur_gnss_pvt_flags = ev->gnss_data.latest.pvt_flags;
		if (ev->gnss_data.fix_ok) {
			shared_state.cur_gnss_samples++;
			shared_state.cur_height_max =
				MAX(shared_state.cur_height_max, ev->gnss_data.latest.height);
			shared_state.cur_height_min =
				MIN(shared_state.cur_height_min, ev->gnss_data.latest.height);
			shared_state.cur_height_sum += (int32_t)ev->gnss_data.latest.height;
			shared_state.cur_speed_max =
				MAX(shared_state.cur_speed_max, ev->gnss_data.latest.speed);
			shared_state.cur_speed_min =
				MIN(shared_state.cur_speed_min, ev->gnss_data.latest.speed);
			shared_state.cur_speed_sum += (uint32_t)ev->gnss_data.latest.speed;
		}
		return false;
	} else if (is_gnss_set_mode_event(eh)) {
		struct gnss_set_mode_event *ev = cast_gnss_set_mode_event(eh);
		switch (ev->mode) {
		case GNSSMODE_INACTIVE:
			shared_state.cur_gnss_modes.usOffCount++;
			break;
		case GNSSMODE_PSM:
			shared_state.cur_gnss_modes.usPSMCount++;
			break;
		case GNSSMODE_CAUTION:
			shared_state.cur_gnss_modes.usCautionCount++;
			break;
		case GNSSMODE_MAX:
			shared_state.cur_gnss_modes.usMaxCount++;
			break;
		default:
			shared_state.cur_gnss_modes.usUnknownCount++;
		}
		return false;
	} else if (is_modem_state(eh)) {
		struct modem_state *ev = cast_modem_state(eh);
		shared_state.cur_modem_pwr_m = ev->mode;
		return false;
	} else if (is_pwr_status_event(eh)) {
		struct pwr_status_event *ev = cast_pwr_status_event(eh);
		if (ev->pwr_state != PWR_CHARGING) {
			/* Add battery max/min voltage [centi-voltage] */
			shared_state.qc_battery.usVbattMax =
				MAX(shared_state.qc_battery.usVbattMax, ev->battery_mv_max / 10);
			shared_state.qc_battery.usVbattMin =
				MIN(shared_state.qc_battery.usVbattMin, ev->battery_mv_min / 10);
		}
		return false;
	}

	else if (is_xy_location(eh)) {
		struct xy_location *ev = cast_xy_location(eh);
		shared_state.pos_count++;
		shared_state.cur_fresh_pos[0] = ev->x;
		shared_state.cur_fresh_pos[1] = ev->y;
		return false;
	}

	else if (is_save_histogram(eh)) {
		save_and_reset = true;
		return false;
	} else {
		/* If event is unhandled, unsubscribe. */
		__ASSERT_NO_MSG(false);
		return false;
	}
}

static bool event_handler(const struct event_header *eh)
{
	k_mutex_lock(&update_in_progress, K_FOREVER);
	bool ret = event_handler_impl(eh);
	k_mutex_unlock(&update_in_progress);
	return ret;
}

EVENT_LISTENER(MODULE, event_handler);

EVENT_SUBSCRIBE(MODULE, zone_change);
EVENT_SUBSCRIBE(MODULE, xy_location);

EVENT_SUBSCRIBE(MODULE, update_collar_status);
EVENT_SUBSCRIBE(MODULE, update_fence_status);

EVENT_SUBSCRIBE(MODULE, activity_level);
EVENT_SUBSCRIBE(MODULE, step_counter_event);

EVENT_SUBSCRIBE(MODULE, gnss_data);
EVENT_SUBSCRIBE(MODULE, gnss_set_mode_event);

EVENT_SUBSCRIBE(MODULE, modem_state);

EVENT_SUBSCRIBE(MODULE, pwr_status_event);

EVENT_SUBSCRIBE(MODULE, save_histogram);

static _Noreturn void collect_stats_fn(void)
{
	static bool has_way_pnt = false;
	static int16_t way_pnt[2];
	static _HISTOGRAM_ANIMAL_BEHAVE animal_behave;
	static uint32_t steps;
	static uint32_t elapsed_time_ms;
	static uint32_t resting;
	static uint32_t walking;
	static uint32_t running;
	static uint32_t grazing;
	static uint32_t unknown;
	static uint32_t ccsleep;
	static uint32_t ccbeacon;
	static uint32_t modem_active;
	static uint32_t insleep;
	static uint32_t inbeacon;
	static uint32_t maxzone;
	static uint32_t cautionzone;
	static uint32_t psmzone;
	static uint32_t nozone;
	static uint32_t psm_state_POT_ms;
	static uint32_t psm_state_Tracking_ms;
	static uint32_t psm_state_Acquisition_ms;
	static uint32_t psm_state_Continous_ms;
	static uint16_t gnss_unknown_count;
	static uint16_t gnss_off_count;
	static uint16_t gnss_psm_count;
	static uint16_t gnss_caution_count;
	static uint16_t gnss_max_count;

	static int16_t gps_height_max = INT16_MIN;
	static int16_t gps_height_min = INT16_MAX;
	static uint16_t speed_min = UINT16_MAX;
	static uint16_t speed_max = 0;
	static uint32_t timeuse_sample_gps;
	static uint32_t speedsum;
	static int32_t heightsum;

	static uint16_t v_bat_min = UINT16_MAX;
	static uint16_t v_bat_max;

	static int64_t uptime;

	while (true) {
		elapsed_time_ms += (uint32_t)k_uptime_delta(&uptime);
		if (elapsed_time_ms >= CONFIG_TIME_USE_RESOLUTION_MS) {
			struct shared_state copied_state;
			/* Thread safe copy/reset the state set in event-thread */
			k_mutex_lock(&update_in_progress, K_FOREVER);
			memcpy(&copied_state, &shared_state, sizeof(struct shared_state));
			reset_state(&shared_state);
			k_mutex_unlock(&update_in_progress);
			/* Local variables from this point */

			gps_height_max = MAX(gps_height_max, copied_state.cur_height_max);
			gps_height_min = MIN(gps_height_min, copied_state.cur_height_min);
			speed_min = MIN(speed_min, copied_state.cur_speed_min);
			speed_max = MAX(speed_max, copied_state.cur_speed_max);
			timeuse_sample_gps += copied_state.cur_gnss_samples;
			speedsum += copied_state.cur_speed_sum;
			heightsum += copied_state.cur_height_sum;

			bool has_dist = copied_state.pos_count > 0 && has_way_pnt;
			int16_t distance =
				has_dist ? TimeuseDistance(way_pnt, copied_state.cur_fresh_pos) : 0;

			if (copied_state.cur_activity_level == ACTIVITY_LOW) {
				animal_behave.has_usRestingTime = true;
				resting += elapsed_time_ms;
			} else if (copied_state.cur_activity_level == ACTIVITY_MED) {
				animal_behave.has_usWalkingTime = true;
				walking += elapsed_time_ms;
				animal_behave.has_usWalkingDist =
					has_dist || animal_behave.has_usWalkingDist;
				animal_behave.usWalkingDist += distance;
			} else if (copied_state.cur_activity_level == ACTIVITY_HIGH) {
				animal_behave.has_usRunningTime = true;
				running += elapsed_time_ms;
				animal_behave.has_usRunningDist =
					has_dist || animal_behave.has_usRunningDist;
				animal_behave.usRunningDist += distance;
			} else {
				animal_behave.has_usUnknownTime = true;
				unknown += elapsed_time_ms;
			}
			if (copied_state.pos_count > 0) {
				has_way_pnt = true;
				way_pnt[0] = copied_state.cur_fresh_pos[0];
				way_pnt[1] = copied_state.cur_fresh_pos[1];
			}

			/* Histogram to predict Current profile of the collar  */
			uint8_t cur_psm_state = (copied_state.cur_gnss_pvt_flags >> 2) & 0x07;
			if (copied_state.cur_collar_status == CollarStatus_Sleep ||
			    copied_state.cur_collar_status == CollarStatus_OffAnimal) {
				ccsleep += elapsed_time_ms;
			} else if (copied_state.cur_fence_status == FenceStatus_BeaconContact ||
				   copied_state.cur_fence_status ==
					   FenceStatus_BeaconContactNormal) {
				ccbeacon += elapsed_time_ms;
			} else if (cur_psm_state == POWER_OPTIMIZED_TRACKING) {
				psm_state_POT_ms += elapsed_time_ms;
			} else if (cur_psm_state == TRACKING) {
				psm_state_Tracking_ms += elapsed_time_ms;
			} else if (cur_psm_state == ACQUSITION || cur_psm_state == ENABLED) {
				psm_state_Acquisition_ms += elapsed_time_ms;
			} else if (cur_psm_state == DISABLED &&
				   copied_state.cur_gnss_pwr_m != GNSSMODE_INACTIVE) {
				psm_state_Continous_ms += elapsed_time_ms;
			} else if (copied_state.cur_gnss_pwr_m == GNSSMODE_INACTIVE) {
				ccsleep += elapsed_time_ms;
			}
			if (copied_state.cur_modem_pwr_m == POWER_ON) {
				modem_active += elapsed_time_ms;
			}
			bool in_beacon_or_sleep = false;
			/* Histogram for different zone  */
			if (copied_state.cur_collar_status == CollarStatus_Sleep ||
			    copied_state.cur_collar_status == CollarStatus_OffAnimal) {
				insleep += elapsed_time_ms;
				in_beacon_or_sleep = true;
			}
			if (copied_state.cur_fence_status == FenceStatus_BeaconContact ||
			    copied_state.cur_fence_status == FenceStatus_BeaconContactNormal) {
				inbeacon += elapsed_time_ms;
				in_beacon_or_sleep = true;
			}

			if (!in_beacon_or_sleep) {
				/* Time use in the different zones */
				if (copied_state.cur_zone == WARN_ZONE ||
				    copied_state.cur_zone == PREWARN_ZONE) {
					maxzone += elapsed_time_ms;
				} else if (copied_state.cur_zone == CAUTION_ZONE) {
					cautionzone += elapsed_time_ms;
				} else if (copied_state.cur_zone == PSM_ZONE) {
					psmzone += elapsed_time_ms;
				} else if (copied_state.cur_zone == NO_ZONE) {
					nozone += elapsed_time_ms;
				}
			}
			steps += copied_state.cur_steps;
			gnss_unknown_count += copied_state.cur_gnss_modes.usUnknownCount;
			gnss_off_count += copied_state.cur_gnss_modes.usOffCount;
			gnss_psm_count += copied_state.cur_gnss_modes.usPSMCount;
			gnss_caution_count += copied_state.cur_gnss_modes.usCautionCount;
			gnss_max_count += copied_state.cur_gnss_modes.usMaxCount;

			v_bat_max = MAX(v_bat_max, copied_state.qc_battery.usVbattMax);
			v_bat_min = MIN(v_bat_min, copied_state.qc_battery.usVbattMin);

			/* Reset timer */
			elapsed_time_ms = 0;
		}
		if (save_and_reset) {
			save_and_reset = false;
			collar_histogram histogram;
			memset(&histogram, 0, sizeof(struct collar_histogram));
			memcpy(&histogram.animal_behave, &animal_behave,
			       sizeof(histogram.animal_behave));

			histogram.animal_behave.usRestingTime = resting / MSEC_PER_SEC;
			histogram.animal_behave.usWalkingTime = walking / MSEC_PER_SEC;
			histogram.animal_behave.usRunningTime = running / MSEC_PER_SEC;
			histogram.animal_behave.usGrazingTime = grazing / MSEC_PER_SEC;
			histogram.animal_behave.usUnknownTime = unknown / MSEC_PER_SEC;
			/* Add stepcounter value */
			histogram.animal_behave.has_usStepCounter = true;
			histogram.animal_behave.usStepCounter = steps;

			histogram.current_profile.usCC_Sleep = ccsleep / MSEC_PER_SEC;
			histogram.current_profile.usCC_BeaconZone = ccbeacon / MSEC_PER_SEC;
			histogram.current_profile.usCC_GNSS_MAX =
				psm_state_Continous_ms / MSEC_PER_SEC;
			histogram.current_profile.usCC_GNSS_SuperE_Acquition =
				psm_state_Acquisition_ms / MSEC_PER_SEC;
			histogram.current_profile.usCC_GNSS_SuperE_Tracking =
				psm_state_Tracking_ms / MSEC_PER_SEC;
			histogram.current_profile.usCC_GNSS_SuperE_POT =
				psm_state_POT_ms / MSEC_PER_SEC;
			histogram.current_profile.usCC_Modem_Active = modem_active / MSEC_PER_SEC;
			histogram.in_zone.usInSleepTime = insleep / MSEC_PER_SEC;
			histogram.in_zone.usBeaconZoneTime = inbeacon / MSEC_PER_SEC;
			histogram.in_zone.usMAXZoneTime = maxzone / MSEC_PER_SEC;
			histogram.in_zone.usCAUTIONZoneTime = cautionzone / MSEC_PER_SEC;
			histogram.in_zone.usPSMZoneTime = psmzone / MSEC_PER_SEC;
			histogram.in_zone.usNOZoneTime = nozone / MSEC_PER_SEC;

			/* due to a typo in the legacy protobuf file, we must convert 16 bit signed to unsigned */
			histogram.qc_baro_gps_max_mean_min.usGpsHeightMax =
				clamp_int16(gps_height_max);
			histogram.qc_baro_gps_max_mean_min.usGpsHeightMin =
				clamp_int16(gps_height_min);
			histogram.qc_baro_gps_max_mean_min.usGpsSpeedMax = speed_max;
			histogram.qc_baro_gps_max_mean_min.usGpsSpeedMin = speed_min;

			if (timeuse_sample_gps > 0) {
				histogram.qc_baro_gps_max_mean_min.usGpsSpeedMean =
					speedsum / timeuse_sample_gps;
				histogram.qc_baro_gps_max_mean_min.usGpsHeightMean =
					heightsum / timeuse_sample_gps;
			}

			histogram.gnss_modes.usUnknownCount = gnss_unknown_count;
			histogram.gnss_modes.usOffCount = gnss_off_count;
			histogram.gnss_modes.usPSMCount = gnss_psm_count;
			histogram.gnss_modes.usCautionCount = gnss_caution_count;
			histogram.gnss_modes.usMaxCount = gnss_max_count;

			histogram.qc_battery.usVbattMax = v_bat_max;
			histogram.qc_battery.usVbattMin = v_bat_min;

			resting = 0;
			walking = 0;
			running = 0;
			grazing = 0;
			unknown = 0;
			ccsleep = 0;
			ccbeacon = 0;
			psm_state_Continous_ms = 0;
			psm_state_Acquisition_ms = 0;
			psm_state_Tracking_ms = 0;
			psm_state_POT_ms = 0;
			modem_active = 0;
			insleep = 0;
			inbeacon = 0;
			maxzone = 0;
			cautionzone = 0;
			psmzone = 0;
			nozone = 0;
			gnss_unknown_count = 0;
			gnss_off_count = 0;
			gnss_psm_count = 0;
			gnss_caution_count = 0;
			gnss_max_count = 0;
			gps_height_min = INT16_MAX;
			gps_height_max = INT16_MIN;
			speed_min = UINT16_MAX;
			speed_max = 0;
			speedsum = 0;
			heightsum = 0;
			timeuse_sample_gps = 0;
			v_bat_max = 0;
			v_bat_min = UINT16_MAX;
			has_way_pnt = false;
			memset(&animal_behave, 0, sizeof(animal_behave));

			/* write temporary variable to queue*/
			while (k_msgq_put(&histogram_msgq, &histogram, K_NO_WAIT) != 0) {
				/* TODO: handle previous histogram not
				 * consumed! */
				k_msgq_purge(&histogram_msgq);
			}
		}
		k_sleep(K_MSEC(250));
	}
}
