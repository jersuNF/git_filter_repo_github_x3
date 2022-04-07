/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include <stdlib.h>
#include <string.h>
#include "messaging_module_events.h"
#include "messaging.h"
#include <logging/log.h>
#include "ble_ctrl_event.h"
#include "ble_data_event.h"
#include "ble_cmd_event.h"
#include "nofence_service.h"
#include "lte_proto_event.h"

#include "cellular_controller_events.h"
#include "gnss_controller_events.h"
#include "request_events.h"
#include "nf_version.h"
#include "collar_protocol.h"
#include "ble_beacon_event.h"

#include "error_event.h"
#include "helpers.h"
#include <power/reboot.h>

#include "nf_crc16.h"

#include "storage_event.h"

#include "storage.h"

#include "pasture_structure.h"
#include "fw_upgrade_events.h"
#include "sound_event.h"

#define TIME_USE_THREAD_PRIORITY CONFIG_TIME_USE_THREAD_PRIORITY
#define resolution_sec CONFIG_TIME_USE_RESOLUTION_SEC
#define MODULE time_use
LOG_MODULE_REGISTER(MODULE, CONFIG_TIME_USE_LOG_LEVEL);

K_THREAD_STACK_DEFINE(collect_stats_stack,
		      CONFIG_TIME_USE_THREAD_STACK_SIZE);
struct k_thread collect_stats_thread;

void collect_stats(void);

K_MUTEX_DEFINE(update_in_progress);

typedef struct histograms
{
	_HISTOGRAM_ANIMAL_BEHAVE animal_behave;
	_HISTOGRAM_ZONE in_zone;
	_POSITION_QC_MAX_MIN_MEAN qc_baro_gps_max_mean_min;
	_HISTOGRAM_CURRENT_PROFILE current_profile;
	_BATTERY_QC qc_battery;
} histograms;

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

void collect_stats(void){
	static uint16_t t;
	uint16_t elapsed = gpt_elapsed16leapSec(t);
	bool sleeporbeacon = false;

	while (true){
		if (elapsed >= resolution_sec){


		}
	}
}

typedef enum {
	ACTIVITY_NO = 0,
	ACTIVITY_LOW = 1,
	ACTIVITY_MED = 2,
	ACTIVITY_HIGH = 3
} acc_activity_t;


typedef enum { /*TODO: move to gnss controller*/
	ACTIVITY_NO = 0,
	ACTIVITY_LOW = 1,
	ACTIVITY_MED = 2,
	ACTIVITY_HIGH = 3
} gnss_pwr_mode;

typedef enum { /*TODO: move to cellular_controller*/
	ACTIVITY_NO = 0,
	ACTIVITY_LOW = 1,
	ACTIVITY_MED = 2,
	ACTIVITY_HIGH = 3
} modem_pwr_mode;

static amc_zone_t cur_zone;
static Mode cur_collar_mode;
static CollarStatus cur_collar_status;
static FenceStatus cur_fence_status;
static bool in_beacon;
static movement_state_t cur_mv_state;
static acc_activity_t cur_activity_level = ACTIVITY_NO;
static uint32_t steps;
static uint32_t xloc, yloc;
static enum gnss_pwr_mode cur_gnss_pwr_m;
static enum modem_pwr_mode cur_modem_pwr_m;
static uint16_t cur_bat_volt;

static bool event_handler(const struct event_header *eh)
{
	if (is_amc_zone_changed(eh)) {
		struct amc_zone_changed *ev =
			cast_cellular_proto_in_event(eh);
		cur_zone = ev.zone;
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
		cur_mv_state = ev.movement_state_t;
		return false;
	}
	if (is_activity_level(eh)) {
		struct activity_level *ev = cast_activity_level(eh);
		cur_activity_level = ev.acc_activity_t;
		return false;
	}
	if (is_step_counter_event(eh)) {
		struct step_counter *ev = cast_step_counter_event(eh);
		steps += ev->count;
		return false;
	}
	if (is_gnss_data(eh)) {
		struct gnss_data *ev = cast_gnss_data(eh);
		xloc = ev->gnss_data.latest.lat;
		yloc = ev->gnss_data.latest.lon;
		return false;
	}
	if (is_gnss_ps_mode(eh)) {
		struct gnss_ps_mode *ev = cast_gnss_ps_mode(eh);
		cur_gnss_pwr_m = ev.mode;
		return false;
	}
	if (is_modem_ps_mode(eh)) {
		struct gnss_ps_mode *ev = cast_modem_ps_mode(eh);
		cur_modem_pwr_m = ev.mode;
		return false;
	}
	if (is_pwr_status_event(eh)) {
		struct pwr_status_event *ev = cast_pwr_status_event(eh);
		cur_bat_volt = ev.battery_mv;
		return false;
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

EVENT_SUBSCRIBE(MODULE, modem_ps_mode);

EVENT_SUBSCRIBE(MODULE, pwr_status_event);