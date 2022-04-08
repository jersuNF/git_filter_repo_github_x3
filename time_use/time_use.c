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
#include "helpers.h"

#define TIME_USE_THREAD_PRIORITY CONFIG_TIME_USE_THREAD_PRIORITY
#define resolution_msec CONFIG_TIME_USE_RESOLUTION_SEC*1000
#define MODULE time_use
LOG_MODULE_REGISTER(MODULE, CONFIG_TIME_USE_LOG_LEVEL);

K_THREAD_STACK_DEFINE(collect_stats_stack,
		      CONFIG_TIME_USE_THREAD_STACK_SIZE);
struct k_thread collect_stats_thread;

void collect_stats(void);

K_MUTEX_DEFINE(update_in_progress);

struct {
	_HISTOGRAM_ANIMAL_BEHAVE animal_behave;
	_HISTOGRAM_ZONE in_zone;
	_POSITION_QC_MAX_MIN_MEAN qc_baro_gps_max_mean_min;
	_HISTOGRAM_CURRENT_PROFILE current_profile;
	_BATTERY_QC qc_battery;
} histogram;

int time_use_module_init(void)
{
	LOG_INF("Initializing time_use module.");

	k_thread_create(&collect_stats_thread, collect_stats_stack,
			K_THREAD_STACK_SIZEOF(collect_stats_stack),
			(k_thread_entry_t) collect_stats,
			NULL, NULL, NULL,
			K_PRIO_COOP(TIME_USE_THREAD_PRIORITY),
			0, K_NO_WAIT);
	return 0;
}

static amc_zone_t cur_zone;
static Mode cur_collar_mode;
static CollarStatus cur_collar_status;
static FenceStatus cur_fence_status;
static bool in_beacon;
static movement_state_t cur_mv_state;
static acc_activity_t cur_activity_level = ACTIVITY_NO;
static uint32_t steps;
static int16_t xloc, yloc;
static gnss_mode_t cur_gnss_pwr_m = GNSSMODE_NOMODE;
static modem_pwr_mode cur_modem_pwr_m = POWER_ON;
static uint16_t cur_bat_volt, cur_min_bat_volt, cur_max_bat_volt;

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
			in_beacon = true;
			return false;
		}
		in_beacon = false;
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
		steps += ev->steps;
		return false;
	}
	if (is_gnss_data(eh)) {
		struct gnss_data *ev = cast_gnss_data(eh);
		cur_gnss_pwr_m = ev->gnss_data.lastfix.mode;
		return false;
	}
	if (is_modem_state(eh)) {
		struct modem_state *ev = cast_modem_state(eh);
		cur_modem_pwr_m = ev->mode;
		return false;
	}
	if (is_pwr_status_event(eh)) {
		struct pwr_status_event *ev = cast_pwr_status_event(eh);
		cur_bat_volt = ev->battery_mv;
		cur_min_bat_volt = ev->battery_mv;
		cur_max_bat_volt = ev->battery_mv;
		return false;
	}

	if (is_xy_location(eh)) {
		struct xy_location *ev = cast_xy_location(eh);
		xloc = ev->x;
		yloc = ev->y;
	}
	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);
	return false;
}

EVENT_LISTENER(MODULE, event_handler);

EVENT_SUBSCRIBE(MODULE, amc_zone_changed);
EVENT_SUBSCRIBE(MODULE, update_collar_mode);
EVENT_SUBSCRIBE(MODULE, update_collar_status);
EVENT_SUBSCRIBE(MODULE, update_fence_status);

EVENT_SUBSCRIBE(MODULE, ble_beacon_event);

EVENT_SUBSCRIBE(MODULE, movement_out_event);
EVENT_SUBSCRIBE(MODULE, activity_level);
EVENT_SUBSCRIBE(MODULE, step_counter);

EVENT_SUBSCRIBE(MODULE, gnss_data);
EVENT_SUBSCRIBE(MODULE, gnss_ps_mode);

EVENT_SUBSCRIBE(MODULE, modem_state);

EVENT_SUBSCRIBE(MODULE, pwr_status_event);

void collect_stats(void){
	static int64_t elapsed_time = 0;
	elapsed_time = k_uptime_delta(&elapsed_time);
	while (true){
		elapsed_time = k_uptime_delta(&elapsed_time);
		if (elapsed_time >= resolution_msec){
			if(acc_ActivityLevel() == ACTIVITY_LOW) { state = RESTING; }		//Grazing
			else if(acc_ActivityLevel() == ACTIVITY_MED) { state = WALKING; }
			else if(acc_ActivityLevel() == ACTIVITY_HIGH) { state = RUNNING; }
			else state = UNKNOWN;

			switch (state)
			{
			case RESTING:
				histogram_animal_behave.has_usRestingTime = true;
				histogram_animal_behave.usRestingTime += elapsed;
				break;
			case WALKING:
				histogram_animal_behave.has_usWalkingTime = true;
				histogram_animal_behave.usWalkingTime += elapsed;
				histogram_animal_behave.has_usWalkingDist = true;
				histogram_animal_behave.usWalkingDist +=  TimeuseDistance(m_i16_TimeuseWaypoint);
				break;
			case RUNNING:
				histogram_animal_behave.has_usRunningTime = true;
				histogram_animal_behave.usRunningTime += elapsed;
				histogram_animal_behave.has_usRunningDist = true;
				histogram_animal_behave.usRunningDist +=  TimeuseDistance(m_i16_TimeuseWaypoint);
				break;
			case GRAZING:
				histogram_animal_behave.has_usGrazingTime = true;
				histogram_animal_behave.usGrazingTime += elapsed;
				break;
			case UNKNOWN:
				histogram_animal_behave.has_usUnknownTime = true;
				histogram_animal_behave.usUnknownTime += elapsed;
				break;
			}

			m_i16_TimeuseWaypoint[0] = GPS()->X;
			m_i16_TimeuseWaypoint[1] = GPS()->Y;

//******************Add Stepcounter value*************************
			uint16_t stepdiff = (uint16_t)(acc_GetTotalSteps() - m_u32_last_stepcounterval);
			histogram_animal_behave.has_usStepCounter = true;
			histogram_animal_behave.usStepCounter += stepdiff;
			m_u32_last_stepcounterval = acc_GetTotalSteps();

//*****************Histogram to predict Current profile of the collar********************

			if (collarStatus == CollarStatus_Sleep || collarStatus == CollarStatus_OffAnimal) { histogram_current_profile.usCC_Sleep += elapsed; }							//"Ultra" Low power
			else if(fenceStatus == FenceStatus_BeaconContact || fenceStatus == FenceStatus_BeaconContactNormal) { histogram_current_profile.usCC_BeaconZone += elapsed; }	//"Ultra" Low power
			else if(gpsp_getPSMState() == POWER_OPTIMIZED_TRACKING)							{ histogram_current_profile.usCC_GNSS_SuperE_POT += elapsed; }					//Low power
			else if(gpsp_getPSMState() == TRACKING)											{ histogram_current_profile.usCC_GNSS_SuperE_Tracking += elapsed; }				//Med power
			else if((gpsp_getPSMState() == ACQUSITION) || (gpsp_getPSMState() == ENABLED))	{ histogram_current_profile.usCC_GNSS_SuperE_Acquition += elapsed; }			//High power
			else if((gpsp_getPSMState() == DISABLED) && (GPS_GetMode() != GPSMODE_INACTIVE)){ histogram_current_profile.usCC_GNSS_MAX += elapsed; }							//High power
			else if(GPS_GetMode() == GPSMODE_INACTIVE)										{ histogram_current_profile.usCC_Sleep += elapsed; }							//"Ultra" Low power

			if(gsm_isTransmitting())														{ histogram_current_profile.usCC_Modem_Active += elapsed; }						//High power++

//*****************Histogram for different zone********************
			if (collarStatus == CollarStatus_Sleep || collarStatus == CollarStatus_OffAnimal) {
				histogram_in_zone.usInSleepTime += elapsed;
				sleeporbeacon = true;
			}
			if(fenceStatus == FenceStatus_BeaconContact || fenceStatus == FenceStatus_BeaconContactNormal) {
				histogram_in_zone.usBeaconZoneTime += elapsed;
				sleeporbeacon = true;
			}

			if(sleeporbeacon == false)
			{
				//Time use in the different zones
				if (WarnZone() || PreWarnZone())	{ histogram_in_zone.usMAXZoneTime += elapsed; }
				else if (CautionZone())				{ histogram_in_zone.usCAUTIONZoneTime += elapsed; }
				else if (PSMZone())					{ histogram_in_zone.usPSMZoneTime += elapsed; }
				else if (NoZone())					{ histogram_in_zone.usNOZoneTime += elapsed;}
			}

//*****************GPS AND BARO deviation values********************
			if(gpsp_isGpsFixEasy())
			{
				m_u32_timeuse_sample_gps++;
				if(gpsp_getHeightMax() > qc_baro_gps_max_mean_min.usGpsHeightMax)	{ qc_baro_gps_max_mean_min.usGpsHeightMax = gpsp_getHeightMax(); }
				if(gpsp_getHeightMin() < qc_baro_gps_max_mean_min.usGpsHeightMin)	{ qc_baro_gps_max_mean_min.usGpsHeightMin = gpsp_getHeightMin(); }
				m_u32_localmean[0] += gpsp_getHeightMean();
				qc_baro_gps_max_mean_min.usGpsHeightMean = m_u32_localmean[0] / m_u32_timeuse_sample_gps;

				if(gpsp_getSpeedMax() > qc_baro_gps_max_mean_min.usGpsSpeedMax)	{ qc_baro_gps_max_mean_min.usGpsSpeedMax = gpsp_getSpeedMax(); }
				if(gpsp_getSpeedMin() < qc_baro_gps_max_mean_min.usGpsSpeedMin)	{ qc_baro_gps_max_mean_min.usGpsSpeedMin = gpsp_getSpeedMin(); }
				m_u32_localmean[1] += gpsp_getSpeedMean();
				qc_baro_gps_max_mean_min.usGpsSpeedMean = m_u32_localmean[1] / m_u32_timeuse_sample_gps;
			}
		}
	}
}