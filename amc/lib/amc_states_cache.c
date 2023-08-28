#include "nclogs.h"
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
#include "stg_config.h"
#include "gnss.h"
#include "gnss_controller_events.h"
#include "messaging_module_events.h"
#include "error_event.h"
#include "ble_beacon_event.h"
#include "movement_events.h"
#include "movement_controller.h"
#include "pwr_event.h"

/* Mode related variables. */
static uint16_t zap_count_day = 0;
static uint8_t teach_mode_finished = 0;

/* Updated once we enter teach mode. */
static uint32_t teach_mode_saved_warn_cnt = 0;
static uint32_t teach_mode_saved_zap_cnt = 0;

static uint32_t total_warn_cnt = 0;
static uint16_t total_zap_cnt = 0;

/* Fence status related variables. */
static uint8_t pain_cnt_def_free = _PAIN_CNT_DEF_ESCAPED;

/** Runtime, cached variables. Some get their values come from
 *  ext flash on init.
 */
static Mode current_mode = Mode_Mode_UNKNOWN;
static FenceStatus current_fence_status = FenceStatus_FenceStatus_UNKNOWN;
static CollarStatus current_collar_status = CollarStatus_CollarStatus_UNKNOWN;

/* Variable used to check GNSS mode. */
static bool first_time_since_start = true;
static bool has_ever_got_fully_resolved_gnss_time = false;
static int64_t forcegnsstofix_timestamp = 0;

/* Movement controller variable. */
static atomic_t movement_state = ATOMIC_INIT(STATE_NORMAL);

/* Power manager variable. */
static atomic_t power_state = ATOMIC_INIT(PWR_NORMAL);

/* warncount - teachwarncount_saved */
static uint32_t teach_zap_cnt = 0;
static uint32_t teach_warn_cnt = 0;

static uint16_t zap_pain_cnt = 0;

#ifdef CONFIG_TEST
void _test_set_firs_time_since_start(bool v)
{
	first_time_since_start = v;
}
void _test_set_has_ever_got_fully_resolved_gnss_time(bool v)
{
	has_ever_got_fully_resolved_gnss_time = v;
}
#endif

void update_movement_state(movement_state_t state)
{
	atomic_set(&movement_state, state);
}

void update_power_state(enum pwr_state_flag state)
{
	atomic_set(&power_state, state);
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

uint16_t get_zap_pain_cnt(void)
{
	return zap_pain_cnt;
}

void increment_zap_count(void)
{
	int err;
	teach_zap_cnt++;
	zap_pain_cnt++;

	total_zap_cnt++;
	err = stg_config_u16_write(STG_U16_ZAP_CNT_TOT, total_zap_cnt);
	if (err != 0) {
		NCLOG_ERR(AMC_MODULE, TRice( iD( 1913),"err: Could not write zap count total, error %i\n", err));
		return;
	}

	/** @todo Add reset for the day. */
	zap_count_day++;
	err = stg_config_u16_write(STG_U16_ZAP_CNT_DAY, zap_count_day);
	if (err != 0) {
		NCLOG_ERR(AMC_MODULE, TRice( iD( 4365),"err: Could not write zap count day, error %i\n", err));
		return;
	}

	/* Notify server about our recent zap. */
	struct update_zap_count *ev = new_update_zap_count();
	ev->count = total_zap_cnt;
	EVENT_SUBMIT(ev);
}

void reset_zap_count_day()
{
	/** @todo Should it be 0xFF or 0? */
	zap_count_day = 0;
	int err = stg_config_u16_write(STG_U16_ZAP_CNT_DAY, zap_count_day);
	if (err != 0) {
		NCLOG_ERR(AMC_MODULE, TRice( iD( 6756),"err: Could not reset zap count day, error %i\n", err));
		return;
	}
}

void cache_storage_variables(void)
{
	int err = stg_config_u16_read(STG_U16_ZAP_CNT_TOT, &total_zap_cnt);
	if (err != 0) {
		NCLOG_ERR(AMC_MODULE, TRice( iD( 1043),"err: Could not read zap count total, error %i\n", err));
		return;
	}

	err = stg_config_u32_read(STG_U32_WARN_CNT_TOT, &total_warn_cnt);
	if (err != 0) {
		NCLOG_ERR(AMC_MODULE, TRice( iD( 1444),"err: Could not read warn count total, error %i\n", err));
		return;
	}

	err = stg_config_u16_read(STG_U16_ZAP_CNT_DAY, &zap_count_day);
	if (err != 0) {
		NCLOG_ERR(AMC_MODULE, TRice( iD( 3162),"err: Could not read zap count day, error %i\n", err));
		return;
	}
}

void increment_warn_count(void)
{
	teach_warn_cnt++;
	total_warn_cnt++;

	/** @todo Periodic ext flash write ? 
	  * Write everytime total_warn_cnt % n == true? 
	  */

	int err = stg_config_u32_write(STG_U32_WARN_CNT_TOT, total_warn_cnt);
	if (err != 0) {
		NCLOG_ERR(AMC_MODULE, TRice( iD( 2969),"err: Could not write warn count total, error %i\n", err));
		return;
	}
}

static void enter_teach_mode()
{
	int err;

	uint32_t warn_cnt = 0;
	err = stg_config_u32_read(STG_U32_WARN_CNT_TOT, &warn_cnt);
	if (err != 0) {
		NCLOG_ERR(AMC_MODULE, TRice( iD( 1149),"err: Could not read warn count total, error %i\n", err));
	}

	uint16_t zap_cnt = 0;
	err = stg_config_u16_read(STG_U16_ZAP_CNT_TOT, &zap_cnt);
	if (err != 0) {
		NCLOG_ERR(AMC_MODULE, TRice( iD( 2976),"err: Could not read zap count total, error %i\n", err));
	}

	err = stg_config_u8_read(STG_U8_TEACH_MODE_FINISHED, &teach_mode_finished);
	if (err != 0) {
		NCLOG_ERR(AMC_MODULE, TRice( iD( 3338),"err: Could not read teach mode finished, error %i\n", err));
	}
	if (teach_mode_finished != 0) {
		teach_mode_finished = 0;

		err = stg_config_u8_write(STG_U8_TEACH_MODE_FINISHED, teach_mode_finished);
		if (err != 0) {
			NCLOG_ERR(AMC_MODULE, TRice( iD( 7499),"err: Could not write teach mode finished, error %i\n", err));
		}
	}
	teach_mode_saved_warn_cnt = (uint16_t)warn_cnt;
	teach_mode_saved_zap_cnt = (uint16_t)zap_cnt;
}

void force_teach_mode()
{
	int err;
	if (current_mode != Mode_Teach) {
		current_mode = Mode_Teach;

		err = stg_config_u8_write(STG_U8_COLLAR_MODE, (uint8_t)current_mode);
		if (err != 0) {
			NCLOG_ERR(AMC_MODULE, TRice( iD( 4889),"err: Could not write to collar mode, error %i \n", err));
		}

		enter_teach_mode();

		/* Notify server about mode change. */
		struct update_collar_mode *mode_ev = new_update_collar_mode();
		mode_ev->collar_mode = current_mode;
		EVENT_SUBMIT(mode_ev);
	}
}

void init_states_and_variables(void)
{
	cache_storage_variables();

	/* Collar mode. */
	uint8_t collar_mode = 0;
	int err = stg_config_u8_read(STG_U8_COLLAR_MODE, &collar_mode);
	if (err) {
		NCLOG_ERR(AMC_MODULE, TRice( iD( 6365),"err: Could not read collar mode %i\n", err));
		collar_mode = Mode_Mode_UNKNOWN;
	}
	current_mode = (Mode)collar_mode;
	if (current_mode == Mode_Teach) {
		enter_teach_mode();
	}

	/* Read collar status from eeprom */
	uint8_t collar_status = 0;
	err = stg_config_u8_read(STG_U8_COLLAR_STATUS, &collar_status);
	if (err) {
		NCLOG_ERR(AMC_MODULE, TRice( iD( 7618),"err: Could not read collar mode %i\n", err));
		collar_status = CollarStatus_CollarStatus_UNKNOWN;
	}
	current_collar_status = (CollarStatus)collar_status;

	/* Read fence status from eeprom */
	uint8_t fence_status = 0;
	err = stg_config_u8_read(STG_U8_FENCE_STATUS, &fence_status);
	if (err) {
		NCLOG_ERR(AMC_MODULE, TRice( iD( 3880),"err: Could not read fence status, %i!\n", err));
		fence_status = FenceStatus_FenceStatus_UNKNOWN;
	}
	current_fence_status = (FenceStatus)fence_status;

	NCLOG_INF(AMC_MODULE, TRice( iD( 7043),"inf: Cached AMC states: Collarmode %i, collarstatus %i, fencestatus %i\n", current_mode, current_fence_status, current_collar_status));

	NCLOG_INF(AMC_MODULE, TRice( iD( 1449),"inf: Cached AMC variables : ZAP_TOTAL %i, WARN_TOTAL %i, ZAP_DAY %i\n", total_zap_cnt, total_warn_cnt, zap_count_day));

	/* Notify server about different states. */
	struct update_zap_count *ev_zap = new_update_zap_count();
	ev_zap->count = total_zap_cnt;
	EVENT_SUBMIT(ev_zap);

	struct update_fence_status *ev_fstatus = new_update_fence_status();
	ev_fstatus->fence_status = current_fence_status;
	EVENT_SUBMIT(ev_fstatus);

	struct update_collar_status *ev_cmode = new_update_collar_status();
	ev_cmode->collar_status = current_collar_status;
	EVENT_SUBMIT(ev_cmode);

	struct update_collar_mode *ev_cstatus = new_update_collar_mode();
	ev_cstatus->collar_mode = current_mode;
	EVENT_SUBMIT(ev_cstatus);
}

Mode get_mode(void)
{
	return current_mode;
}

Mode calc_mode(void)
{
	int err;
	uint32_t teach_zap_cnt = total_zap_cnt - teach_mode_saved_zap_cnt;
	uint32_t teach_warn_cnt = total_warn_cnt - teach_mode_saved_warn_cnt;

	Mode new_mode = current_mode;
	switch (current_mode) {
	case Mode_Mode_UNKNOWN:
		if (fnc_valid_fence()) {
			NCLOG_INF(AMC_MODULE, TRice0( iD( 2380),"inf: Unknown->Teach\n"));
			new_mode = Mode_Teach;
		} else if (fnc_valid_def()) {
			NCLOG_INF(AMC_MODULE, TRice0( iD( 1296),"inf: Unknown->Trace\n"));
			new_mode = Mode_Trace;
		}
		break;
	case Mode_Teach:
		if (trace_mode_conditions()) {
			NCLOG_INF(AMC_MODULE, TRice0( iD( 5388),"inf: Teach->Trace\n"));
			new_mode = Mode_Trace;
		} else if (teach_zap_cnt >= _TEACHMODE_ZAP_CNT_HIGHLIM) {
			NCLOG_INF(AMC_MODULE, TRice0( iD( 6695),"inf: Teach->Trace\n"));
			new_mode = Mode_Trace;
		} else if ((teach_zap_cnt >= TEACHMODE_ZAP_CNT_LOWLIM) &&
			   ((teach_warn_cnt - teach_zap_cnt) >= _TEACHMODE_WARN_CNT_LOWLIM)) {
			NCLOG_INF(AMC_MODULE, TRice0( iD( 1375),"inf: Teach->Fence\n"));
			/* See https://youtrack.axbit.com/youtrack/issue/NOF-310. */
			new_mode = Mode_Fence;
			teach_mode_finished = 1;
			/** @todo Need to set to 0 when going from
			 *  Fence -> Teach
			 */
			err = stg_config_u8_write(STG_U8_TEACH_MODE_FINISHED, teach_mode_finished);
			if (err != 0) {
				NCLOG_ERR(AMC_MODULE, TRice( iD( 7742),"err: Failed to write to ext flash, id %d, error %d\n", STG_U8_TEACH_MODE_FINISHED, err));
			}
		}
		break;
	case Mode_Fence:
		if (trace_mode_conditions()) {
			NCLOG_INF(AMC_MODULE, TRice0( iD( 1252),"inf: Fence->Trace\n"));
			new_mode = Mode_Trace;
		}
		break;
	case Mode_Trace:
		if (!trace_mode_conditions()) {
			if (teach_mode_finished) {
				NCLOG_INF(AMC_MODULE, TRice0( iD( 5471),"inf: Trace->Fence\n"));
				new_mode = Mode_Fence;
			} else {
				NCLOG_INF(AMC_MODULE, TRice0( iD( 7458),"inf: Trace->Teach\n"));
				new_mode = Mode_Teach;
			}
		}
		break;
	default:
		new_mode = Mode_Mode_UNKNOWN;
		NCLOG_INF(AMC_MODULE, TRice0( iD( 6996),"inf: Unknown\n"));
		break;
	}

	/* If new mode, write to ext flash storage. */
	if (current_mode != new_mode) {
		current_mode = new_mode;

		err = stg_config_u8_write(STG_U8_COLLAR_MODE, (uint8_t)new_mode);
		if (err != 0) {
			NCLOG_ERR(AMC_MODULE, TRice( iD( 4467),"err: Failed to write new collar mode to ext flash, error %i \n", err));
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
	return fnc_valid_fence() /*&& gpsp_isGpsFresh()*/ && gnss_has_accepted_fix() &&
	       !(cur_zone == WARN_ZONE || cur_zone == NO_ZONE);
}

FenceStatus get_fence_status(void)
{
	return current_fence_status;
}

CollarStatus get_collar_status(void)
{
	return current_collar_status;
}

FenceStatus calc_fence_status(uint32_t maybe_out_of_fence, enum beacon_status_type beacon_status)
{
	FenceStatus new_fence_status = current_fence_status;
	uint32_t maybe_out_of_fence_delta =
		((k_uptime_get_32() - maybe_out_of_fence) / MSEC_PER_SEC);

	switch (current_fence_status) {
	case FenceStatus_FenceStatus_UNKNOWN: {
		if (beacon_status == BEACON_STATUS_REGION_NEAR) {
			new_fence_status = FenceStatus_BeaconContact;
			NCLOG_INF(AMC_MODULE, TRice0( iD( 2281),"inf: FenceStatus:Unknown->BeaconContact\n"));
		} else if (fnc_valid_def()) {
			new_fence_status = FenceStatus_NotStarted;
			NCLOG_INF(AMC_MODULE, TRice0( iD( 6986),"inf: FenceStatus:Unknown->NotStarted\n"));
		}
		break;
	}
	case FenceStatus_NotStarted: {
		if (beacon_status == BEACON_STATUS_REGION_NEAR) {
			new_fence_status = FenceStatus_BeaconContact;
			NCLOG_INF(AMC_MODULE, TRice0( iD( 7684),"inf: FenceStatus:NotStarted->BeaconContact\n"));
		} else if (is_inside_fence_relaxed()) {
			new_fence_status = FenceStatus_FenceStatus_Normal;
			NCLOG_INF(AMC_MODULE, TRice0( iD( 7077),"inf: FenceStatus:NotStarted->Normal\n"));
		}
		break;
	}
	case FenceStatus_FenceStatus_Normal: {
		if (beacon_status == BEACON_STATUS_REGION_NEAR) {
			new_fence_status = FenceStatus_BeaconContactNormal;
			NCLOG_INF(AMC_MODULE, TRice0( iD( 3939),"inf: FenceStatus:Normal->BeaconContactNormal\n"));
		} else if (maybe_out_of_fence_delta > OUT_OF_FENCE_TIME) {
			/** Old @todo ? UBX_Poll(UBXID_MON_HW);
				 * v3.21-7: Poll hardware info (fex. jamming). 
				 */
			new_fence_status = FenceStatus_MaybeOutOfFence;
			NCLOG_INF(AMC_MODULE, TRice0( iD( 3031),"inf: FenceStatus:Normal->MaybeOutsideFence\n"));
		} else if (zap_pain_cnt >= pain_cnt_def_free) {
			new_fence_status = FenceStatus_Escaped;
			NCLOG_INF(AMC_MODULE, TRice0( iD( 2168),"inf: NFenceStatus:ormal->Escaped\n"));
		}
		break;
	}
	case FenceStatus_MaybeOutOfFence: {
		if (beacon_status == BEACON_STATUS_REGION_NEAR) {
			new_fence_status = FenceStatus_BeaconContact;
			NCLOG_INF(AMC_MODULE, TRice0( iD( 4366),"inf: FenceStatus:MaybeOutside->BeaconContact\n"));
		} else if (zap_pain_cnt >= pain_cnt_def_free) {
			new_fence_status = FenceStatus_Escaped;
			NCLOG_INF(AMC_MODULE, TRice0( iD( 5759),"inf: FenceStatus:MaybeOutside->Escaped\n"));
		} else if (maybe_out_of_fence_delta < OUT_OF_FENCE_TIME) {
			new_fence_status = FenceStatus_FenceStatus_Normal;
			NCLOG_INF(AMC_MODULE, TRice0( iD( 7731),"inf: FenceStatus:MaybeOutside->Normal\n"));
		}
		break;
	}
	case FenceStatus_Escaped: {
		if (beacon_status == BEACON_STATUS_REGION_NEAR) {
			new_fence_status = FenceStatus_BeaconContact;
			NCLOG_INF(AMC_MODULE, TRice0( iD( 1961),"inf: FenceStatus:Escaped->BeaconContact\n"));
		} else if (is_inside_fence_relaxed()) {
			new_fence_status = FenceStatus_FenceStatus_Normal;
			NCLOG_INF(AMC_MODULE, TRice0( iD( 2521),"inf: FenceStatus:Escaped->Normal\n"));
		}
		break;
	}
	case FenceStatus_BeaconContact: {
		if (beacon_status != BEACON_STATUS_REGION_NEAR) {
			if (fnc_valid_fence()) {
				new_fence_status = FenceStatus_NotStarted;
				NCLOG_INF(AMC_MODULE, TRice0( iD( 6133),"inf: FenceStatus:BeaconContact->NotStarted\n"));
			} else {
				new_fence_status = FenceStatus_FenceStatus_UNKNOWN;
				NCLOG_INF(AMC_MODULE, TRice0( iD( 2667),"inf: FenceStatus:BeaconContact->Unknown\n"));
			}
		}
		break;
	}
	case FenceStatus_BeaconContactNormal: {
		if (beacon_status != BEACON_STATUS_REGION_NEAR) {
			new_fence_status = FenceStatus_FenceStatus_Normal;
			NCLOG_INF(AMC_MODULE, TRice0( iD( 3788),"inf: FenceStatus:BeaconContactNormal->Normal\n"));
		}
		break;
	}
	case FenceStatus_FenceStatus_Invalid: {
		NCLOG_INF(AMC_MODULE, TRice0( iD( 6028),"inf: FenceStatus:Invalid\n"));
		break;
	}
	case FenceStatus_TurnedOffByBLE: {
		NCLOG_INF(AMC_MODULE, TRice0( iD( 2163),"inf: FenceStatus:Fence turned of by BLE\n"));
		break;
	}
	default: {
		new_fence_status = FenceStatus_FenceStatus_UNKNOWN;
		NCLOG_INF(AMC_MODULE, TRice0( iD( 4453),"inf: FenceStatus:?->Unknown\n"));
		char *msg = "Unknown fence status received.";
		nf_app_error(ERR_AMC, -EINVAL, msg, strlen(msg));
		break;
	}
	}

	/* If new status, write to ext flash storage */
	if (current_fence_status != new_fence_status) {
		current_fence_status = new_fence_status;

		int err = stg_config_u8_write(STG_U8_FENCE_STATUS, (uint8_t)current_fence_status);
		if (err != 0) {
			NCLOG_ERR(AMC_MODULE, TRice( iD( 1493),"err: Failed to write new fence status to ext flash, error %i \n", err));
		}

		/* Notify server about fence status change. */
		struct update_fence_status *fence_ev = new_update_fence_status();
		fence_ev->fence_status = current_fence_status;
		EVENT_SUBMIT(fence_ev);

		/* Notify server if animal escaped. */
		if (current_fence_status == FenceStatus_Escaped) {
			struct animal_escape_event *ev = new_animal_escape_event();
			EVENT_SUBMIT(ev);
		}
	}
	return new_fence_status;
}

int force_fence_status(FenceStatus new_fence_status)
{
	if ((new_fence_status != FenceStatus_FenceStatus_UNKNOWN) &&
	    (new_fence_status != FenceStatus_NotStarted) &&
	    (new_fence_status != FenceStatus_FenceStatus_Invalid) &&
	    (new_fence_status != FenceStatus_TurnedOffByBLE)) {
		return -EACCES;
	}

	if (new_fence_status != current_fence_status) {
		if ((new_fence_status == FenceStatus_NotStarted) && (fnc_valid_def() != true)) {
			NCLOG_WRN(AMC_MODULE, TRice0( iD( 3407),"wrn: Unable to force fence status\n"));
			return -EACCES;
		}

		/* Write new fence status to ext flash storage */
		int err = stg_config_u8_write(STG_U8_FENCE_STATUS, (uint8_t)new_fence_status);
		if (err != 0) {
			NCLOG_WRN(AMC_MODULE, TRice( iD( 7258),"wrn: Failed to write new fence status to ext flash, error:%d\n", err));
			return -EACCES;
		}

		/* Update current fence status */
		current_fence_status = new_fence_status;

		/* Notify listeners about fence status change. */
		struct update_fence_status *event = new_update_fence_status();
		event->fence_status = current_fence_status;
		EVENT_SUBMIT(event);
	}
	return 0;
}

CollarStatus calc_collar_status(void)
{
	movement_state_t mov_state = atomic_get(&movement_state);
	CollarStatus new_collar_status = current_collar_status;

	switch (current_collar_status) {
	case CollarStatus_CollarStatus_UNKNOWN: {
		if (mov_state == STATE_NORMAL) {
			new_collar_status = CollarStatus_CollarStatus_Normal;
			NCLOG_INF(AMC_MODULE, TRice0( iD( 1373),"inf: CollarStatus:Unknown->Normal\n"));
		} else if (mov_state == STATE_SLEEP) {
			new_collar_status = CollarStatus_Sleep;
			NCLOG_INF(AMC_MODULE, TRice0( iD( 6407),"inf: CollarStatus:Unknown->Sleep\n"));
		}
		break;
	}
	case CollarStatus_CollarStatus_Normal: {
		if (mov_state == STATE_SLEEP) {
			new_collar_status = CollarStatus_Sleep;
			NCLOG_INF(AMC_MODULE, TRice0( iD( 4776),"inf: CollarStatus:Normal->Sleep\n"));
		} else if (mov_state == STATE_INACTIVE) {
			NCLOG_WRN(AMC_MODULE, TRice0( iD( 5688),"wrn: CollarStatus:Went directly to inactive in normal\n"));
		}
		break;
	}
	case CollarStatus_Sleep: {
		if (mov_state == STATE_NORMAL) {
			new_collar_status = CollarStatus_CollarStatus_Normal;
			NCLOG_INF(AMC_MODULE, TRice0( iD( 7009),"inf: CollarStatus:Sleep->Normal\n"));
		} else if (mov_state == STATE_INACTIVE) {
			new_collar_status = CollarStatus_OffAnimal;
			NCLOG_INF(AMC_MODULE, TRice0( iD( 2817),"inf: CollarStatus:Sleep->OffAnimal\n"));
		}
		break;
	}
	case CollarStatus_OffAnimal: {
		if (mov_state == STATE_NORMAL) {
			new_collar_status = CollarStatus_CollarStatus_Normal;
			NCLOG_INF(AMC_MODULE, TRice0( iD( 6375),"inf: CollarStatus:Off->Normal\n"));
		} else if (mov_state == STATE_SLEEP) {
			new_collar_status = CollarStatus_Sleep;
			NCLOG_INF(AMC_MODULE, TRice0( iD( 3402),"inf: CollarStatus:Off->Sleep\n"));
		}
		break;
	}
	case CollarStatus_PowerOff: {
		if (atomic_get(&power_state) != PWR_CRITICAL) {
			if (mov_state == STATE_NORMAL) {
				new_collar_status = CollarStatus_CollarStatus_Normal;
				NCLOG_INF(AMC_MODULE, TRice0( iD( 3183),"inf: CollarStatus:PowerOff->Normal\n"));
			} else if (mov_state == STATE_SLEEP) {
				new_collar_status = CollarStatus_Sleep;
				NCLOG_INF(AMC_MODULE, TRice0( iD( 3866),"inf: CollarStatus:PowerOff->Sleep\n"));
			} else if (mov_state == STATE_INACTIVE) {
				char *msg = "CollarStatus:Went directly to inactive from powerOff";
				nf_app_error(ERR_AMC, -EINVAL, msg, strlen(msg));
			} else {
				new_collar_status = CollarStatus_CollarStatus_UNKNOWN;
				NCLOG_INF(AMC_MODULE, TRice0( iD( 7961),"inf: CollarStatus:PowerOff->UNKNOWN\n"));
			}
		}
		break;
	}
	default: {
		new_collar_status = CollarStatus_CollarStatus_UNKNOWN;
		char *msg = "Unknown collar status";
		nf_app_error(ERR_AMC, -EINVAL, msg, strlen(msg));
		break;
	}
	}

	/* Added to enable power off mode without power switch. 
	 * Trigged by low battery voltage. */
	if (atomic_get(&power_state) == PWR_CRITICAL) {
		new_collar_status = CollarStatus_PowerOff;
		NCLOG_INF(AMC_MODULE, TRice0( iD( 1035),"inf: CollarStatus:...->PowerOff\n"));
	}

	/* If new status, write to ext flash storage. */
	if (current_collar_status != new_collar_status) {
		current_collar_status = new_collar_status;

		int err = stg_config_u8_write(STG_U8_COLLAR_STATUS, (uint8_t)current_collar_status);
		if (err != 0) {
			NCLOG_ERR(AMC_MODULE, TRice( iD( 4199),"err: Failed to write new collar status to ext flash, error %i \n", err));
		}

		/* Notify server about collar status change. */
		struct update_collar_status *collar_ev = new_update_collar_status();
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

void set_sensor_modes(Mode mode, FenceStatus fs, CollarStatus cs, amc_zone_t zone)
{
	uint8_t gnss_mode = GNSSMODE_CAUTION;
	has_ever_got_fully_resolved_gnss_time |= gnss_is_time_fully_resolved();

	if (cs == CollarStatus_Sleep || cs == CollarStatus_OffAnimal ||
	    fs == FenceStatus_BeaconContact || fs == FenceStatus_BeaconContactNormal) {
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
			if (gnss_is_time_fully_resolved()) {
				first_time_since_start = false;
			}
		}
		/* Timeout this function after 1 hour. */
		uint32_t delta_fix =
			(uint32_t)((k_uptime_get() - forcegnsstofix_timestamp) / MSEC_PER_SEC);
		if (delta_fix >= 3600) {
			first_time_since_start = false;
		}
	}

	if (get_correction_status() > 0) {
		/* GNSSMODE_MAX is needed whenever the warning tone is playing. */
		gnss_mode = GNSSMODE_MAX;
	}

	/** @todo [LEGACY] NOF-512 Always set backupmode 
	  * when in undervoltage state. 
	  */
	if (atomic_get(&power_state) == PWR_CRITICAL) {
		gnss_mode = GNSSMODE_INACTIVE;
	}

	/* If we are in PSM mode and the GNSS receiver still hasn't delivered
	 * fullyResolvedTime since MCU boot, we need to keep the receiver
	 * in continous mode
	 */
	if (gnss_mode == GNSSMODE_PSM) {
		if (!has_ever_got_fully_resolved_gnss_time) {
			gnss_mode = GNSSMODE_CAUTION;
		}
	}

	/* Send GNSS mode change event from amc_gnss.c */
	if (gnss_get_mode() != gnss_mode) {
		gnss_update_mode(gnss_mode);
	}
}
