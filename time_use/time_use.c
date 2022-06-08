/*
 * Copyright (c) 2021 Nofence AS
 */
#include <stdio.h>
#include <zephyr.h>
#include <logging/log.h>
#include "nofence_service.h"
#include "gnss_controller_events.h"
#include "cellular_controller_events.h"
#include "request_events.h"
#include "nf_version.h"
#include "collar_protocol.h"
#include "ble_beacon_event.h"
#include "error_event.h"
#include "movement_events.h"
#include "request_events.h"
#include <power/reboot.h>
#include "amc_events.h"
#include "messaging_module_events.h"
#include "movement_events.h"
#include "pwr_event.h"
#include "amc_gnss.h"
#include "histogram_events.h"
#include "time_use.h"
#include "time_use_helpers.h"

#define TIME_USE_THREAD_PRIORITY CONFIG_TIME_USE_THREAD_PRIORITY
#define MODULE time_use
LOG_MODULE_REGISTER(MODULE, CONFIG_TIME_USE_LOG_LEVEL);

K_THREAD_STACK_DEFINE(collect_stats_stack, CONFIG_TIME_USE_THREAD_STACK_SIZE);
struct k_thread collect_stats_thread;

void collect_stats(void);

K_MUTEX_DEFINE(update_in_progress);

collar_histogram histogram;
K_MSGQ_DEFINE(histogram_msgq, sizeof(struct collar_histogram), 1, 4);

int time_use_module_init(void)
{
	LOG_INF("Initializing the time_use module.");
	k_thread_create(&collect_stats_thread, collect_stats_stack,
			K_THREAD_STACK_SIZEOF(collect_stats_stack),
			(k_thread_entry_t)collect_stats, NULL, NULL, NULL,
			TIME_USE_THREAD_PRIORITY, 0, K_NO_WAIT);
	return 0;
}
typedef enum { RESTING, GRAZING, WALKING, RUNNING, UNKNOWN } STATE;

static STATE cur_animal_state = UNKNOWN;
static amc_zone_t cur_zone;
static Mode cur_collar_mode;
static CollarStatus cur_collar_status;
static FenceStatus cur_fence_status;
static bool in_beacon_or_sleep, save_and_reset;
static movement_state_t cur_mv_state;
static acc_activity_t cur_activity_level = ACTIVITY_NO;
static uint32_t steps, steps_old;
int16_t m_i16_way_pnt[2], fresh_pos[2];
static gnss_mode_t cur_gnss_pwr_m = GNSSMODE_NOMODE;
static modem_pwr_mode cur_modem_pwr_m = POWER_ON;
static uint32_t m_u32_timeuse_sample_gps = 0;
static int16_t m_i16_heightmax = INT16_MIN;
static int16_t m_i16_heightmin = INT16_MAX;
static int32_t m_i32_heightmean = 0;
static uint16_t m_u16_speedmin = UINT16_MAX;
static uint16_t m_u16_speedmax = 0;
static uint32_t m_u32_speedmean = 0;
static uint16_t m_ui16_hs_samples = 0;

static bool event_handler(const struct event_header *eh)
{
	if (is_zone_change(eh)) {
		struct zone_change *ev = cast_zone_change(eh);
		cur_zone = ev->zone;
		return false;
	}
	if (is_update_collar_mode(eh)) {
		struct update_collar_mode *ev = cast_update_collar_mode(eh);
		cur_collar_mode = ev->collar_mode;
		return false;
	}
	if (is_update_collar_status(eh)) {
		struct update_collar_status *ev = cast_update_collar_status(eh);
		cur_collar_status = ev->collar_status;
		return false;
	}
	if (is_update_fence_status(eh)) {
		struct update_fence_status *ev = cast_update_fence_status(eh);
		cur_fence_status = ev->fence_status;
		return false;
	}
	if (is_ble_beacon_event(eh)) {
		struct ble_beacon_event *ev = cast_ble_beacon_event(eh);
		if (ev->status == BEACON_STATUS_REGION_NEAR ||
		    ev->status == BEACON_STATUS_REGION_FAR) {
			in_beacon_or_sleep = true;
			return false;
		}
		in_beacon_or_sleep = false;
		return false;
	}
	if (is_movement_out_event(eh)) {
		struct movement_out_event *ev = cast_movement_out_event(eh);
		cur_mv_state = ev->state;
		return false;
	}
	if (is_activity_level(eh)) {
		struct activity_level *ev = cast_activity_level(eh);
		cur_activity_level = ev->level;
		return false;
	}
	if (is_step_counter_event(eh)) {
		struct step_counter_event *ev = cast_step_counter_event(eh);
		steps = ev->steps;
		return false;
	}
	if (is_gnss_data(eh)) {
		struct gnss_data *ev = cast_gnss_data(eh);
		cur_gnss_pwr_m = ev->gnss_data.lastfix.mode;
		if (ev->gnss_data.fix_ok) {
			if (ev->gnss_data.latest.height > m_i16_heightmax) {
				m_i16_heightmax = ev->gnss_data.latest.height;
			} //Find min max value of height and speed. reset upon each request of this parameters
			if (ev->gnss_data.latest.height < m_i16_heightmin) {
				m_i16_heightmin = ev->gnss_data.latest.height;
			}
			m_i32_heightmean +=
				(int32_t)ev->gnss_data.latest.height;

			if (ev->gnss_data.latest.speed > m_u16_speedmax) {
				m_u16_speedmax = ev->gnss_data.latest.speed;
			}
			if (ev->gnss_data.latest.speed < m_u16_speedmin) {
				m_u16_speedmin = ev->gnss_data.latest.speed;
			}
			m_u32_speedmean += (uint32_t)ev->gnss_data.latest.speed;

			m_ui16_hs_samples++;
		}
		return false;
	}
	if (is_modem_state(eh)) {
		struct modem_state *ev = cast_modem_state(eh);
		cur_modem_pwr_m = ev->mode;
		return false;
	}
	if (is_pwr_status_event(eh)) {
		struct pwr_status_event *ev = cast_pwr_status_event(eh);
		if (ev->pwr_state == PWR_BATTERY) {
			/* Add battery max/min voltage [centi-voltage] */
			histogram.qc_battery.usVbattMax =
				(uint16_t)(ev->battery_mv_max / 10);
			histogram.qc_battery.usVbattMin =
				(uint16_t)(ev->battery_mv_min / 10);
			LOG_INF("Save to histogram.qc_battery: max: %u, min: %u",
				histogram.qc_battery.usVbattMax,
				histogram.qc_battery.usVbattMin);
		}
		return false;
	}

	if (is_xy_location(eh)) {
		struct xy_location *ev = cast_xy_location(eh);
		fresh_pos[0] = ev->x;
		fresh_pos[1] = ev->y;
		return false;
	}

	if (is_save_histogram(eh)) {
		save_and_reset = true;
		return false;
	}
	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);
	return false;
}

EVENT_LISTENER(MODULE, event_handler);

EVENT_SUBSCRIBE(MODULE, amc_zone_changed);
EVENT_SUBSCRIBE(MODULE, xy_location);
EVENT_SUBSCRIBE(MODULE, update_collar_mode);
EVENT_SUBSCRIBE(MODULE, update_collar_status);
EVENT_SUBSCRIBE(MODULE, update_fence_status);

EVENT_SUBSCRIBE(MODULE, ble_beacon_event);

EVENT_SUBSCRIBE(MODULE, movement_out_event);
EVENT_SUBSCRIBE(MODULE, activity_level);
EVENT_SUBSCRIBE(MODULE, step_counter_event);

EVENT_SUBSCRIBE(MODULE, gnss_data);
EVENT_SUBSCRIBE(MODULE, gnss_ps_mode);

EVENT_SUBSCRIBE(MODULE, modem_state);

EVENT_SUBSCRIBE(MODULE, pwr_status_event);

EVENT_SUBSCRIBE(MODULE, save_histogram);

void collect_stats(void)
{
	static int64_t elapsed_time_sec, uptime;
	while (true) {
		elapsed_time_sec += (int64_t)(k_uptime_delta(&uptime) / 1000);
		if (elapsed_time_sec >= CONFIG_TIME_USE_RESOLUTION_SEC &&
		    !save_and_reset) {
			if (cur_activity_level == ACTIVITY_LOW) {
				cur_animal_state = RESTING;
			} //Grazing
			else if (cur_activity_level == ACTIVITY_MED) {
				cur_animal_state = WALKING;
			} else if (cur_activity_level == ACTIVITY_HIGH) {
				cur_animal_state = RUNNING;
			} else
				cur_mv_state = UNKNOWN;

			switch (cur_animal_state) {
			case RESTING:
				histogram.animal_behave.has_usRestingTime =
					true;
				histogram.animal_behave.usRestingTime +=
					elapsed_time_sec;
				break;
			case WALKING:
				histogram.animal_behave.has_usWalkingTime =
					true;
				histogram.animal_behave.usWalkingTime +=
					elapsed_time_sec;
				histogram.animal_behave.has_usWalkingDist =
					true;
				histogram.animal_behave.usWalkingDist +=
					TimeuseDistance(m_i16_way_pnt,
							fresh_pos);
				break;
			case RUNNING:
				histogram.animal_behave.has_usRunningTime =
					true;
				histogram.animal_behave.usRunningTime +=
					elapsed_time_sec;
				histogram.animal_behave.has_usRunningDist =
					true;
				histogram.animal_behave.usRunningDist +=
					TimeuseDistance(m_i16_way_pnt,
							fresh_pos);
				break;
			case GRAZING:
				histogram.animal_behave.has_usGrazingTime =
					true;
				histogram.animal_behave.usGrazingTime +=
					elapsed_time_sec;
				break;
			case UNKNOWN:
				histogram.animal_behave.has_usUnknownTime =
					true;
				histogram.animal_behave.usUnknownTime +=
					elapsed_time_sec;
				break;
			}

			m_i16_way_pnt[0] = fresh_pos[0];
			m_i16_way_pnt[1] = fresh_pos[1];

			//******************Add Stepcounter value*************************
			uint16_t stepdiff = (uint16_t)(steps - steps_old);
			histogram.animal_behave.has_usStepCounter = true;
			histogram.animal_behave.usStepCounter += stepdiff;
			steps_old = steps;

			//*****************Histogram to predict Current profile of the collar********************

			if (cur_collar_status == CollarStatus_Sleep ||
			    cur_collar_status == CollarStatus_OffAnimal) {
				histogram.current_profile.usCC_Sleep +=
					elapsed_time_sec;
			} //"Ultra" Low power
			else if (cur_fence_status ==
					 FenceStatus_BeaconContact ||
				 cur_fence_status ==
					 FenceStatus_BeaconContactNormal) {
				histogram.current_profile.usCC_BeaconZone +=
					elapsed_time_sec;
			} //"Ultra" Low power
			/*TODO: fix when GNSS modes are available. */
			//			else if(cur_gnss_pwr_m == POWER_OPTIMIZED_TRACKING){
			//				histogram.current_profile
			//					.usCC_GNSS_SuperE_POT += elapsed_time_sec; }					//Low power
			//			else if(cur_gnss_pwr_m == TRACKING){ histogram_current_profile.usCC_GNSS_SuperE_Tracking += elapsed_time_sec; }				//Med power
			//			else if((cur_gnss_pwr_m == ACQUSITION) ||
			//				 (gpsp_getPSMState() == ENABLED))	{ histogram.current_profile.usCC_GNSS_SuperE_Acquition += elapsed_time_sec; }			//High power
			//			else if((cur_gnss_pwr_m == DISABLED) && (GPS_GetMode
			//								  () != GPSMODE_INACTIVE)){ histogram.current_profile.usCC_GNSS_MAX += elapsed_time_sec; }							//High power
			//			else if(GPS_GetMode() == GPSMODE_INACTIVE)
			//			{
			//				histogram.current_profile.usCC_Sleep +=
			//					elapsed_time_sec;
			//			} //"Ultra" Low power

			if (cur_modem_pwr_m == POWER_ON) {
				histogram.current_profile.usCC_Modem_Active +=
					elapsed_time_sec;
			} //High power++

			//*****************Histogram for different zone********************
			if (cur_collar_status == CollarStatus_Sleep ||
			    cur_collar_status == CollarStatus_OffAnimal) {
				histogram.in_zone.usInSleepTime +=
					elapsed_time_sec;
				in_beacon_or_sleep = true;
			}
			if (cur_fence_status == FenceStatus_BeaconContact ||
			    cur_fence_status ==
				    FenceStatus_BeaconContactNormal) {
				histogram.in_zone.usBeaconZoneTime +=
					elapsed_time_sec;
				in_beacon_or_sleep = true;
			}

			if (in_beacon_or_sleep == false) {
				//Time use in the different zones
				if (cur_zone == WARN_ZONE ||
				    cur_zone == PREWARN_ZONE) {
					histogram.in_zone.usMAXZoneTime +=
						elapsed_time_sec;
				} else if (cur_zone == CAUTION_ZONE) {
					histogram.in_zone.usCAUTIONZoneTime +=
						elapsed_time_sec;
				} else if (cur_zone == PSM_ZONE) {
					histogram.in_zone.usPSMZoneTime +=
						elapsed_time_sec;
				} else if (cur_zone == NO_ZONE) {
					histogram.in_zone.usNOZoneTime +=
						elapsed_time_sec;
				}
			}

			//*****************GPS AND BARO deviation values********************
			if (gnss_has_easy_fix()) {
				m_u32_timeuse_sample_gps++;
				if (m_i16_heightmax >
				    histogram.qc_baro_gps_max_mean_min
					    .usGpsHeightMax) {
					histogram.qc_baro_gps_max_mean_min
						.usGpsHeightMax =
						m_i16_heightmax;
				}
				if (m_i16_heightmin <
				    histogram.qc_baro_gps_max_mean_min
					    .usGpsHeightMin) {
					histogram.qc_baro_gps_max_mean_min
						.usGpsHeightMin =
						m_i16_heightmin;
				}

				histogram.qc_baro_gps_max_mean_min
					.usGpsHeightMean =
					m_i32_heightmean /
					m_u32_timeuse_sample_gps;

				if (m_u16_speedmax >
				    histogram.qc_baro_gps_max_mean_min
					    .usGpsSpeedMax) {
					histogram.qc_baro_gps_max_mean_min
						.usGpsSpeedMax = m_u16_speedmax;
				}
				if (m_u16_speedmin <
				    histogram.qc_baro_gps_max_mean_min
					    .usGpsSpeedMin) {
					histogram.qc_baro_gps_max_mean_min
						.usGpsSpeedMin = m_u16_speedmin;
				}
				histogram.qc_baro_gps_max_mean_min
					.usGpsSpeedMean =
					m_u32_speedmean /
					m_u32_timeuse_sample_gps;
			}
		}
		if (save_and_reset) {
			save_and_reset = false;
			/*write to queue*/
			while (k_msgq_put(&histogram_msgq, &histogram,
					  K_NO_WAIT) != 0) {
				/* TODO: handle previous histogram not
				 * consumed! */
				k_msgq_purge(&histogram_msgq);
			}
			memset(&histogram, 0, sizeof(struct collar_histogram));
			m_u32_timeuse_sample_gps = 0;
			m_i16_heightmax = INT16_MIN;
			m_i16_heightmin = INT16_MAX;
			m_i32_heightmean = 0;
			m_u16_speedmin = UINT16_MAX;
			m_u16_speedmax = 0;
			m_u32_speedmean = 0;
			m_ui16_hs_samples = 0;
		}
		k_sleep(K_MSEC(150));
	}
}