/*
 * Copyright (c) 2022 Nofence AS
 */

#include <zephyr.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(amc_states, CONFIG_AMC_LIB_LOG_LEVEL);

#include "amc_cache.h"
#include "amc_states.h"
#include "embedded.pb.h"
#include "pasture_structure.h"
#include "amc_zone.h"
#include "amc_gnss.h"
#include "amc_const.h"

#include "ble_beacon_event.h"

/** @todo Store/fetch to/from EEPROM. */
/* Mode related variables. */
static uint16_t zap_count_day = 0;
static bool teach_mode_finished = false;
static uint16_t teach_mode_saved_warn_cnt = 0;
static uint16_t teach_mode_saved_zap_cnt = 0;
static uint16_t total_zap_cnt;

/* Fence status related variables. */
static FenceStatus current_fence_status;
/** @todo Can be updated from server? */
static uint8_t pain_cnt_def_free = _PAIN_CNT_DEF_ESCAPED;

/** END eeprom @todo. **/

/** Runtime, cached variables. Some get their values from
 *  eeprom on init.
 */
static Mode current_mode = Mode_Mode_UNKNOWN;
static uint16_t teach_zap_cnt = 0;
static uint16_t teach_warn_cnt = 0;
static uint16_t zap_pain_cnt = 0;

atomic_t current_beacon_status = ATOMIC_INIT(BEACON_STATUS_NOT_FOUND);

static bool trace_mode_conditions()
{
	if (!fnc_any_valid_fence()) {
		return true;
	}

	if (zap_count_day >= MAX_PAIN_ONE_DAY) {
		return true;
	}

	return false;
}

void reset_zap_pain_cnt(void)
{
	zap_pain_cnt = 0;
}

/** @todo Move these to another amc_* file? */
void increment_zap_count(void)
{
	/** @todo Write some of these to eeprom */
	teach_zap_cnt++;
	zap_pain_cnt++;

	total_zap_cnt++;

	/** @todo Add reset for the day. */
	zap_count_day++;
}

void init_eeprom_variables(void)
{
	/** @todo fetch from eeprom. */
	total_zap_cnt = 0;
}

void increment_warn_count(void)
{
	teach_warn_cnt++;
}

static void enter_teach_mode()
{
	/** @todo fetch from eeprom. */
	teach_mode_saved_warn_cnt = 0; /* EEPROM_TEACH_MODE_WARN_COUNT. */
	teach_mode_saved_zap_cnt = 0; /* EEPROM_TEACH_MODE_ZAP_COUNT. */
	teach_mode_finished = false;
}

void init_mode_status(void)
{
	/* Mode. */
	/** @todo Set current_mode to value in eeprom. */
	// current_mode = eeprom_mode
	if (current_mode == Mode_Teach) {
		enter_teach_mode();
	}
}

Mode get_mode(void)
{
	return current_mode;
}

Mode calc_mode(void)
{
	/* Should not need to fetch from eeprom or other place since
	 * we can simply cache the mode for faster access.
	 * The init_mode_status can set the current_mode cache.
	 */
	Mode new_mode = current_mode;
	switch (current_mode) {
	case Mode_Mode_UNKNOWN:
		if (fnc_any_valid_fence()) {
			new_mode = Mode_Teach;
		} else if (fnc_valid_def()) {
			new_mode = Mode_Trace;
		}
		break;
	case Mode_Teach:
		if (trace_mode_conditions()) {
			new_mode = Mode_Trace;
		} else if (teach_zap_cnt >= _TEACHMODE_ZAP_CNT_HIGHLIM) {
			new_mode = Mode_Trace;
		} else if ((teach_zap_cnt >= TEACHMODE_ZAP_CNT_LOWLIM) &&
			   ((teach_warn_cnt - teach_zap_cnt) >=
			    _TEACHMODE_WARN_CNT_LOWLIM)) {
			/* See https://youtrack.axbit.com/youtrack/issue/NOF-310. */
			new_mode = Mode_Fence;
			teach_mode_finished = true;
		}
		break;
	case Mode_Fence:
		if (trace_mode_conditions()) {
			new_mode = Mode_Trace;
		}
		break;
	case Mode_Trace:
		if (!trace_mode_conditions()) {
			/** @todo Fetch from eeprom. */
			if (teach_mode_finished) {
				new_mode = Mode_Fence;
			} else {
				new_mode = Mode_Teach;
			}
		}
		break;
	default:
		new_mode = Mode_Mode_UNKNOWN;
		break;
	}
	return new_mode;
}

static inline bool is_inside_fence_relaxed()
{
	amc_zone_t cur_zone = zone_get();
	/** @todo gpsp_isGpsFresh()??????? */
	return fnc_any_valid_fence() /*&& gpsp_isGpsFresh()*/ &&
	       gnss_has_accepted_fix() &&
	       !(cur_zone == WARN_ZONE || cur_zone == NO_ZONE);
}

void set_beacon_status(enum beacon_status_type status)
{
	atomic_set(&current_beacon_status, status);
	LOG_DBG("Updated beacon status to enum ID %i", status);
}

FenceStatus calc_fence_status(uint16_t maybe_out_of_fence_timestamp)
{
	FenceStatus new_fence_status = current_fence_status;

	enum beacon_status_type beacon_status =
		atomic_get(&current_beacon_status);

	uint32_t maybe_out_of_fence_delta = (k_uptime_get_32() / MSEC_PER_SEC) -
					    maybe_out_of_fence_timestamp;

	switch (current_fence_status) {
	case FenceStatus_FenceStatus_UNKNOWN:
		if (beacon_status == BEACON_STATUS_REGION_NEAR) {
			new_fence_status = FenceStatus_BeaconContact;
			LOG_INF("Unknown->BeaconContact");
		} else if (fnc_valid_def()) {
			new_fence_status = FenceStatus_NotStarted;
			LOG_INF("Unknown->NotStarted");
		}
		break;
	case FenceStatus_NotStarted:
		if (beacon_status == BEACON_STATUS_REGION_NEAR) {
			new_fence_status = FenceStatus_BeaconContact;
			LOG_INF("NotStarted->BeaconContact");
		} else if (is_inside_fence_relaxed()) {
			new_fence_status = FenceStatus_FenceStatus_Normal;
			LOG_INF("NotStarted->Normal");
		}
		break;
	case FenceStatus_FenceStatus_Normal:
		if (beacon_status == BEACON_STATUS_REGION_NEAR) {
			new_fence_status = FenceStatus_BeaconContactNormal;
			LOG_INF("Normal->BeaconContactNormal");
		} else if (maybe_out_of_fence_delta > OUT_OF_FENCE_TIME) {
			/** Old @todo ? UBX_Poll(UBXID_MON_HW);
			 * v3.21-7: Poll hardware info (fex. jamming). 
			 */
			new_fence_status = FenceStatus_MaybeOutOfFence;
			LOG_INF("Normal->MaybeOutsideFence");
		} else if (zap_pain_cnt >= pain_cnt_def_free) {
			new_fence_status = FenceStatus_Escaped;
			LOG_INF("Normal->Escaped");
		}
		break;
	case FenceStatus_MaybeOutOfFence:
		if (beacon_status == BEACON_STATUS_REGION_NEAR) {
			new_fence_status = FenceStatus_BeaconContact;
			LOG_INF("MaybeOutside->BeaconContact");
		} else if (zap_pain_cnt >= pain_cnt_def_free) {
			new_fence_status = FenceStatus_Escaped;
			LOG_INF("MaybeOutside->Escaped");
		} else if (maybe_out_of_fence_delta < OUT_OF_FENCE_TIME) {
			new_fence_status = FenceStatus_FenceStatus_Normal;
			LOG_INF("MaybeOutside->Normal");
		}
		break;
	case FenceStatus_Escaped:
		if (beacon_status == BEACON_STATUS_REGION_NEAR) {
			new_fence_status = FenceStatus_BeaconContact;
			LOG_INF("Escaped->BeaconContact");
		} else if (is_inside_fence_relaxed()) {
			new_fence_status = FenceStatus_FenceStatus_Normal;
			LOG_INF("Escaped->Normal");
		}
		break;

	case FenceStatus_BeaconContact:
		if (beacon_status != BEACON_STATUS_REGION_NEAR) {
			if (fnc_any_valid_fence()) {
				new_fence_status = FenceStatus_NotStarted;
				LOG_INF("BeaconContact->NotStarted");
			} else {
				new_fence_status =
					FenceStatus_FenceStatus_UNKNOWN;
				LOG_INF("BeaconContact->Unknown");
			}
		}
		break;
	case FenceStatus_BeaconContactNormal:
		if (beacon_status != BEACON_STATUS_REGION_NEAR) {
			new_fence_status = FenceStatus_FenceStatus_Normal;
			LOG_INF("BeaconContactNormal->Normal");
		}
		break;
	case FenceStatus_FenceStatus_Invalid:
		LOG_INF("Invalid fence status.");
		/** @todo Fence is invalid, nothing much to do at this point, 
		 * wait for fenceDef to be downloaded again.
		 */
		break;
	case FenceStatus_TurnedOffByBLE:
		LOG_INF("Fence turned of by BLE.");
		/** @todo Fence must be downloaded again. */
		break;
	default:
		/** Internal error. @todo Error handle? */
		new_fence_status = FenceStatus_FenceStatus_UNKNOWN;
		LOG_INF("?->Unknown");
		break;
	}
	return new_fence_status;
}

CollarStatus calc_collar_status(void)
{
	/** @todo Add collar status logic based on movement controller. */
	return CollarStatus_Stuck;
}

int set_sensor_modes(Mode amc_mode, gnss_mode_t gnss_mode, FenceStatus fs,
		     CollarStatus cs)
{
	/** @todo Set STUFF!!!! */
	return 0;
}