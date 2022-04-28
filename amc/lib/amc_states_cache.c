/*
 * Copyright (c) 2022 Nofence AS
 */

#include <zephyr.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(amc_states, CONFIG_AMC_LIB_LOG_LEVEL);

#include "amc_cache.h"
#include "amc_states_cache.h"
#include "embedded.pb.h"
#include "pasture_structure.h"
#include "amc_zone.h"
#include "amc_gnss.h"
#include "amc_const.h"
#include "amc_correction.h"
#include "nf_settings.h"
#include "gnss.h"
#include "gnss_controller_events.h"
#include "messaging_module_events.h"

#include "ble_beacon_event.h"
#include "movement_events.h"

#include "movement_controller.h"

/* Mode related variables. */
static uint16_t zap_count_day = 0;
static uint8_t teach_mode_finished = 0;

/* Updated once we enter teach mode. */
static uint16_t teach_mode_saved_warn_cnt = 0;
static uint16_t teach_mode_saved_zap_cnt = 0;

static uint32_t total_warn_cnt;
static uint16_t total_zap_cnt;

/* Fence status related variables. */
static uint8_t pain_cnt_def_free = _PAIN_CNT_DEF_ESCAPED;

/** Runtime, cached variables. Some get their values come from
 *  eeprom on init.
 */
static Mode current_mode = Mode_Mode_UNKNOWN;
static FenceStatus current_fence_status = FenceStatus_FenceStatus_UNKNOWN;
static CollarStatus current_collar_status = CollarStatus_CollarStatus_UNKNOWN;
static gnss_mode_t current_gnss_mode = GNSSMODE_NOMODE;

/* Variable used to check GNSS mode. */
static bool first_time_since_start = true;
static int64_t forcegnsstofix_timestamp = 0;

/* Movement controller variable. */
static atomic_t movement_state = ATOMIC_INIT(STATE_NORMAL);

/* warncount - teachwarncount_saved */
static uint16_t teach_zap_cnt = 0;
static uint16_t teach_warn_cnt = 0;

static uint16_t zap_pain_cnt = 0;

static atomic_t current_beacon_status = ATOMIC_INIT(BEACON_STATUS_NOT_FOUND);

void update_movement_state(movement_state_t state)
{
	atomic_set(&movement_state, state);
}

static bool trace_mode_conditions()
{
	if (!fnc_valid_fence()) {
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

void increment_zap_count(void)
{
	teach_zap_cnt++;
	zap_pain_cnt++;

	total_zap_cnt++;
	int err = eep_uint16_write(EEP_ZAP_CNT_TOT, total_zap_cnt);

	if (err) {
		LOG_ERR("Could not write zap count total %i", err);
		return;
	}

	/** @todo Add reset for the day. */
	zap_count_day++;
	err = eep_uint16_write(EEP_ZAP_CNT_DAY, zap_count_day);

	if (err) {
		LOG_ERR("Could not write zap count day %i", err);
		return;
	}
}

void reset_zap_count_day()
{
	/** @todo Should it be 0xFF or 0? */
	zap_count_day = 0;
	int err = eep_uint16_write(EEP_ZAP_CNT_DAY, zap_count_day);

	if (err) {
		LOG_ERR("Could not reset zap count day %i", err);
		return;
	}
}

void cache_eeprom_variables(void)
{
	int err = eep_uint16_read(EEP_ZAP_CNT_TOT, &total_zap_cnt);

	if (err) {
		LOG_ERR("Could not read zap count total %i", err);
		return;
	}

	err = eep_uint32_read(EEP_WARN_CNT_TOT, &total_warn_cnt);

	if (err) {
		LOG_ERR("Could not read warn count total %i", err);
		return;
	}

	err = eep_uint16_read(EEP_ZAP_CNT_DAY, &zap_count_day);

	if (err) {
		LOG_ERR("Could not read zap count day %i", err);
		return;
	}
}

void increment_warn_count(void)
{
	teach_warn_cnt++;
	total_warn_cnt++;

	/** @todo Periodic eeprom write ? 
	  * Write everytime total_warn_cnt % n == true? 
	  */
	int err = eep_uint32_write(EEP_WARN_CNT_TOT, total_warn_cnt);

	if (err) {
		LOG_ERR("Could not write warn count total %i", err);
		return;
	}
}

static void enter_teach_mode()
{
	int err;

	uint32_t warn_cnt = 0;
	uint16_t zap_cnt = 0;

	err = eep_uint32_read(EEP_WARN_CNT_TOT, &warn_cnt);

	if (err) {
		LOG_ERR("Could not read warn count total %i", err);
		return;
	}

	err = eep_uint16_read(EEP_ZAP_CNT_TOT, &zap_cnt);

	if (err) {
		LOG_ERR("Could not read zap count total %i", err);
		return;
	}

	err = eep_uint8_read(EEP_TEACH_MODE_FINISHED, &teach_mode_finished);

	if (err) {
		LOG_ERR("Could not read teach mode finished %i", err);
		return;
	}

	teach_mode_saved_warn_cnt = (uint16_t)warn_cnt;
	teach_mode_saved_zap_cnt = (uint8_t)zap_cnt;
}

void init_states_and_variables(void)
{
	cache_eeprom_variables();
	uint8_t status_code = 0;

	/* Collar mode. */
	int err = eep_uint8_read(EEP_COLLAR_MODE, &status_code);
	if (err) {
		LOG_ERR("Could not read collar mode %i", err);
		status_code = Mode_Mode_UNKNOWN;
	}
	current_mode = (Mode)status_code;

	if (current_mode == Mode_Teach) {
		enter_teach_mode();
	}

	/* Fence status. */
	err = eep_uint8_read(EEP_FENCE_STATUS, &status_code);

	if (err) {
		LOG_ERR("Could not read fence status %i", err);
		status_code = FenceStatus_FenceStatus_UNKNOWN;
	}
	current_fence_status = (FenceStatus)status_code;

	/* Collar status. */
	err = eep_uint8_read(EEP_COLLAR_STATUS, &status_code);
	if (err) {
		LOG_ERR("Could not read collar status %i", err);
		status_code = CollarStatus_CollarStatus_UNKNOWN;
	}
	current_collar_status = (CollarStatus)status_code;

	LOG_INF("Cached AMC states: Collarmode %i, collarstatus %i, fencestatus %i",
		current_mode, current_fence_status, current_collar_status);

	LOG_INF("Cached AMC variables : ZAP_TOTAL %i, WARN_TOTAL %i, ZAP_DAY %i",
		total_zap_cnt, total_warn_cnt, zap_count_day);
}

Mode get_mode(void)
{
	return current_mode;
}

Mode calc_mode(void)
{
	uint16_t teach_zap_cnt = total_zap_cnt - teach_mode_saved_zap_cnt;
	uint16_t teach_warn_cnt = total_warn_cnt - teach_mode_saved_warn_cnt;

	Mode new_mode = current_mode;
	switch (current_mode) {
	case Mode_Mode_UNKNOWN:
		if (fnc_valid_fence()) {
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
			teach_mode_finished = 1;
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

	/* If new mode, write to EEPROM. */
	if (current_mode != new_mode) {
		current_mode = new_mode;
		int err = eep_uint8_write(EEP_COLLAR_MODE, (uint8_t)new_mode);

		if (err) {
			LOG_ERR("Could not write to collar mode %i ", err);
		}

		/* Notify server about mode change. */
		struct update_collar_mode *mode_ev = new_update_collar_mode();
		mode_ev->collar_mode = current_mode;
		EVENT_SUBMIT(mode_ev);
	}

	return new_mode;
}

static inline bool is_inside_fence_relaxed()
{
	amc_zone_t cur_zone = zone_get();
	/** @todo gpsp_isGpsFresh()??????? */
	return fnc_valid_fence() /*&& gpsp_isGpsFresh()*/ &&
	       gnss_has_accepted_fix() &&
	       !(cur_zone == WARN_ZONE || cur_zone == NO_ZONE);
}

void set_beacon_status(enum beacon_status_type status)
{
	atomic_set(&current_beacon_status, status);
	LOG_DBG("Updated beacon status to enum ID %i", status);
}

FenceStatus get_fence_status(void)
{
	return current_fence_status;
}

CollarStatus get_collar_status(void)
{
	return current_collar_status;
}

FenceStatus calc_fence_status(uint32_t maybe_out_of_fence,
			      enum beacon_status_type beacon_status)
{
	FenceStatus new_fence_status = current_fence_status;

	uint32_t maybe_out_of_fence_delta =
		((uint32_t)(k_uptime_get_32() / MSEC_PER_SEC)) -
		maybe_out_of_fence;

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
			if (fnc_valid_fence()) {
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

	/* If new status, write to EEPROM. */
	if (current_fence_status != new_fence_status) {
		current_fence_status = new_fence_status;
		int err = eep_uint8_write(EEP_FENCE_STATUS,
					  (uint8_t)current_fence_status);

		if (err) {
			LOG_ERR("Could not write to fence status %i ", err);
		}

		/* Notify server about fence status change. */
		struct update_fence_status *fence_ev =
			new_update_fence_status();
		fence_ev->fence_status = current_fence_status;
		EVENT_SUBMIT(fence_ev);
	}

	return new_fence_status;
}

CollarStatus calc_collar_status(void)
{
	movement_state_t mov_state = atomic_get(&movement_state);

	CollarStatus new_collar_status = current_collar_status;

	switch (current_collar_status) {
	case CollarStatus_CollarStatus_UNKNOWN:
		if (mov_state == STATE_NORMAL) {
			new_collar_status = CollarStatus_CollarStatus_Normal;
			LOG_INF("Unknown->Normal");
		} else if (mov_state == STATE_SLEEP) {
			new_collar_status = CollarStatus_Sleep;
			LOG_INF("Unknown->Sleep");
		}
		break;
	case CollarStatus_CollarStatus_Normal:
		/** @todo add nomov !!!! if (nomov_GetStatus()) {
		  *	new_collar_status = CollarStatus_Stuck;
		  *	LOG_INF("Normal->Stuck");
		  * } else 
		  */
		if (mov_state == STATE_SLEEP) {
			new_collar_status = CollarStatus_Sleep;
			LOG_INF("Normal->Sleep");
		} /** else? @todo Should we go directly from normal to inactive? */
		break;
	case CollarStatus_Stuck:
		/** @todo add nomov !!!!
		 * 	if (!nomov_GetStatus()) {
		 *	new_collar_status = CollarStatus_CollarStatus_Normal;
		 *	LOG_INF("Stuck->Normal");
		 * }
		 */
		break;
	case CollarStatus_Sleep:
		if (mov_state == STATE_NORMAL) {
			new_collar_status = CollarStatus_CollarStatus_Normal;
			LOG_INF("Sleep->Normal");
		} else if (mov_state == STATE_INACTIVE) {
			new_collar_status = CollarStatus_OffAnimal;
			LOG_INF("Sleep->OffAnimal");
		}
		break;
	case CollarStatus_OffAnimal:
		if (mov_state == STATE_NORMAL) {
			new_collar_status = CollarStatus_CollarStatus_Normal;
			LOG_INF("Off->Normal");
		} else if (mov_state == STATE_SLEEP) {
			new_collar_status = CollarStatus_Sleep;
			LOG_INF("Off->Sleep");
		}
		break;
	case CollarStatus_PowerOff:
		/** @todo? if (uvlo_gprs_state() == true) { */
		if (mov_state == STATE_NORMAL) {
			new_collar_status = CollarStatus_CollarStatus_Normal;
			LOG_INF("PowerOff->Normal");
		} else if (mov_state == STATE_SLEEP) {
			new_collar_status = CollarStatus_Sleep;
			LOG_INF("PowerOff->Sleep");
		} /** @todo Add this ?? else if (mov_state == STATE_INACTIVE) {
				new_collar_status = CollarStatus_OffAnimal;
				LOG_INF("PowerOff->OffAnimal");
			} */
		else {
			new_collar_status = CollarStatus_CollarStatus_UNKNOWN;
			LOG_INF("PowerOff->UNKNOWN");
		}
		/*}*/
		break;
	default:
		new_collar_status = CollarStatus_CollarStatus_UNKNOWN;
		LOG_INF("?->UNKNOWN");
		break;
	}

	/** @todo do we use this? if (uvlo_gprs_state() ==
	    false) { //Added to enable power off mode without power switch. Trigged by low battery voltage
		new_collar_status = CollarStatus_PowerOff;
		LOG_INF("...->PowerOff");
	}
	*/

	/* If new status, write to EEPROM. */
	if (current_collar_status != new_collar_status) {
		current_collar_status = new_collar_status;
		int err = eep_uint8_write(EEP_COLLAR_STATUS,
					  (uint8_t)current_collar_status);

		if (err) {
			LOG_ERR("Could not write to collar status %i ", err);
		}

		/* Notify server about collar status change. */
		struct update_collar_status *collar_ev =
			new_update_collar_status();
		collar_ev->collar_status = current_collar_status;
		EVENT_SUBMIT(collar_ev);
	}

	return new_collar_status;
}

/* Called everytime we install a new pasture. */
void restart_force_gnss_to_fix(void)
{
	/* NOF-618 trigger to restart the "force GPS to fix" 
	 * also when new pasture is downloaded. 
	 */
	forcegnsstofix_timestamp = k_uptime_get();

	/* NOF-618. */
	first_time_since_start = true;
}

void set_sensor_modes(Mode mode, FenceStatus fs, CollarStatus cs,
		      amc_zone_t zone)
{
	uint8_t gnss_mode = GNSSMODE_CAUTION;

	if (cs == CollarStatus_Sleep || cs == CollarStatus_OffAnimal ||
	    fs == FenceStatus_BeaconContact ||
	    fs == FenceStatus_BeaconContactNormal) {
		gnss_mode = GNSSMODE_INACTIVE;
	} else if (mode == Mode_Teach || mode == Mode_Fence) {
		if (fs == FenceStatus_Escaped) {
			if (gnss_has_accepted_fix()) {
				gnss_mode = GNSSMODE_PSM;
			} else {
				/* Should struggle to get OKFix to possibly 
				 * release the BeenInside or Escaped status.
				 */
				gnss_mode = GNSSMODE_CAUTION;
			}
		} else if (fs == FenceStatus_NotStarted) {
			gnss_mode = GNSSMODE_CAUTION;
		} else {
			if (zone == WARN_ZONE || zone == PREWARN_ZONE) {
				/* [LEGACY] v3.19-7: Moved this if statement up, 
				 * to prevent GPSMODE_ONOFF when in warnzone.
				 */
				gnss_mode = GNSSMODE_MAX;
			} else if (zone == CAUTION_ZONE) {
				gnss_mode = GNSSMODE_CAUTION;
			} else if (zone == PSM_ZONE) {
				/* [LEGACY] PSHUSTAD: See  
				 * http://youtrack.axbit.no/youtrack/issue/NOF-186 
				 */
				gnss_mode = GNSSMODE_PSM;
			}
			/* Nozone. */
			else {
				/* [LEGACY] http://youtrack.axbit.no/youtrack/issue/NOF-156. */
				gnss_mode = GNSSMODE_CAUTION;
			}
		}
	} else if (mode == Mode_Trace) {
		gnss_mode = GNSSMODE_PSM;
	}
	/* If cold-start or new fence definition downloaded, 
	 * first_time_since_start is set. In this case,
	 * try to get a fix for 1 hour unless we are in 
	 * beacon zone or have a fresh fix.
	 */
	if (first_time_since_start) {
		if (!gnss_has_easy_fix() || fs == FenceStatus_NotStarted) {
			if (fs != FenceStatus_BeaconContact &&
			    fs != FenceStatus_BeaconContactNormal) {
				gnss_mode = GNSSMODE_CAUTION;
			}
		} else {
			/* GPS fix recently, do not check anymore. */
			first_time_since_start = false;
		}
		/* Timeout this function after 1 hour. */
		uint32_t delta_fix =
			(uint32_t)((k_uptime_get() - forcegnsstofix_timestamp) /
				   MSEC_PER_SEC);
		if (delta_fix >= 3600) {
			first_time_since_start = false;
		}
	}

	if (get_correction_status() > 0) {
		/* GNSSMODE_MAX is needed whenever the warning tone is playing. */
		gnss_mode = GNSSMODE_MAX;
	}

	/** @todo NOF-512 Always set backupmode when in undervoltage state
	 * if (uvlo_gprs_state() == false) {
	 * 	gnss_mode = GPSMODE_INACTIVE;
	 * }
	 * Do we have uvlo function somewhere?
	 */

	/* Send GNSS mode change event from amc_gnss.c */
	if (current_gnss_mode != gnss_mode) {
		current_gnss_mode = gnss_mode;
		gnss_update_mode(current_gnss_mode);
	}
}