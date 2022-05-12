/*
 * Copyright (c) 2022 Nofence AS
 */

#include <zephyr.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(amc_correction, CONFIG_AMC_LIB_LOG_LEVEL);

#include "amc_correction.h"
#include "amc_states_cache.h"
#include "amc_gnss.h"
#include "amc_const.h"
#include "amc_handler.h"

/* For playing sound and fetching freq limits and zapping. */
#include "sound_event.h"
#include "ep_event.h"

/* Function declarations. */
static void correction_start(int16_t mean_dist);
static void correction_end(void);
static void correction_pause(Reason reason, int16_t mean_dist);
static void correction(Mode amc_mode, int16_t mean_dist, int16_t dist_change);

static bool zap_eval_doing;
static uint32_t zap_timestamp;
static uint32_t correction_pause_timestamp;

/* Distance from fence where the last warning was started. */
static int16_t last_warn_dist;

/* True from function Correction start to correction_end. 
 * Needed for function Correction to perform anything. 
 */
static uint8_t correction_started = 0;

/* True from function correction start to correction_pause. (Sound is ON). */
static uint8_t correction_warn_on = 0;

/* Semaphore used to make correction wait for new freq calculation. */
K_SEM_DEFINE(correction_freq_updated_sem, 0, 1);
K_SEM_DEFINE(correction_warn_sem, 0, 1);
void correction_freq_thread_fn(void);

K_THREAD_DEFINE(correction_freq_thread, CONFIG_CORRECTION_THREAD_STACK_SIZE,
		correction_freq_thread_fn, NULL, NULL, NULL,
		CONFIG_CORRECTION_THREAD_PRIORITY, 0, 0);

/* Atomic variables used by freq calculator thread
 * and corrcetion consumer.
 */

static atomic_t last_warn_freq = ATOMIC_INIT(0);
static atomic_t dist_change_atomic = ATOMIC_INIT(0);

void correction_freq_thread_fn()
{
	while (true) {
		/* Take sem forever, we only want to do this if correction
		 * has started. 
		 */
		k_sem_take(&correction_warn_sem, K_FOREVER);

		Mode mode = get_mode();
		int16_t dist_change = atomic_get(&dist_change_atomic);
		uint16_t freq = atomic_get(&last_warn_freq);
		int16_t inc_tone_slope = 0, dec_tone_slope = 0;

		if (mode == Mode_Teach) {
			inc_tone_slope = TEACHMODE_DIST_INCR_SLOPE_LIM;
			dec_tone_slope = TEACHMODE_DIST_DECR_SLOPE_LIM;
		} else {
			inc_tone_slope = DIST_INCR_SLOPE_LIM;
			dec_tone_slope = DIST_DECR_SLOPE_LIM;
		}

		if (dist_change > inc_tone_slope) {
			freq += WARN_TONE_SPEED_HZ;
		}
		if (dist_change < dec_tone_slope) {
			freq -= WARN_TONE_SPEED_HZ;
		}

		atomic_set(&last_warn_freq, freq);
		k_sem_give(&correction_warn_sem);
		k_sem_give(&correction_freq_updated_sem);
		k_sleep(K_MSEC(WARN_TONE_SPEED_MS));
	}
}

static void correction_start(int16_t mean_dist)
{
	uint32_t delta_correction_pause =
		(k_uptime_get_32() - correction_pause_timestamp) / 1000;

	if (delta_correction_pause > CORRECTION_PAUSE_MIN_TIME) {
		if (!correction_started) {
			atomic_set(&last_warn_freq, WARN_FREQ_INIT);
			last_warn_dist = mean_dist;
			/** @todo log_WriteCorrectionMessage(true); */

			/** @deprecated ??
			 *  g_i16_WarnStartWayP[0] = GPS()->X;
			 *  g_i16_WarnStartWayP[1] = GPS()->Y;
			 */
			LOG_INF("Correction started.");
		}
		if (!correction_warn_on) {
			/** Submit warn zone event to buzzer. 
			 * 
			 * @note This warn zone event 
			 * will play indefinitly
			 * unless one of the three happens and 
			 * it will turn off:
			 * 1. amc_correction doesn't update the 
			 *    warn frequency with a new value 
			 *    within 1 second  timeout (1 sec 
			 *    is the default value at least).
			 * 2. A new sound event has been submitted 
			 *    i.e. FIND_ME, or OFF, in which case 
			 *    we have to resubmit the 
			 *    SND_WARN event. This is currently 
			 *    implemented when we update the freq.
			 *    Where we submit the event if the
			 *    buzzer is IDLE.
			 * 3. It gets a frequency that is outside 
			 *    the freq range for instance below 
			 *    WARN_SOUND_INIT or higher than
			 *    WARN_SOUND_MAX, in which case 
			 *    it turns OFF.
			 */
			struct sound_event *snd_ev = new_sound_event();
			snd_ev->type = SND_WARN;
			EVENT_SUBMIT(snd_ev);

			/** @todo log_WriteCorrectionMessage(true); */
			increment_warn_count();
			LOG_INF("Warn zone counter++ and told buzzer to enter WARN");
		}
		correction_started = 1;
		correction_warn_on = 1;
		k_sem_give(&correction_warn_sem);
	}
}

static void correction_end(void)
{
	if (correction_started && !correction_warn_on) {
		/** Turn off the sound buzzer. @note This will stop
		 * any FIND_ME or other sound events as well. 
		 */
		struct sound_event *snd_ev = new_sound_event();
		snd_ev->type = SND_OFF;
		EVENT_SUBMIT(snd_ev);

		last_warn_dist = LIM_WARN_MIN_DM;
		reset_zap_pain_cnt();

		/** @todo log_WriteCorrectionMessage(false); */

		/** @todo????
		 * #ifdef GSMCONNECT_AFTER_WARN
		 * 		gsm_ConnectAndTransfer(
		 * 			GSM_CALL_CORRECTIONEND); // Makes sure the updated data is transferred immediately when correction ends
		 * #endif
		 */
		correction_started = 0;
		k_sem_take(&correction_warn_sem, K_SECONDS(1));

		LOG_INF("Ended correction.");
	}
}

/** @brief This routine pauses the warning tone and correction process.
 * 
 * @param reason reason for pause.
 * @param mean_dist mean distance calculated from border. Used to update the
 *        new warn area distance reference.
 */
static void correction_pause(Reason reason, int16_t mean_dist)
{
	int16_t dist_add = 0;

	/** Turn off the sound buzzer. @note This will stop
	 * any FIND_ME or other sound events as well. 
	 */
	struct sound_event *snd_ev = new_sound_event();
	snd_ev->type = SND_OFF;
	EVENT_SUBMIT(snd_ev);

	LOG_INF("Paused correction warning due to reason %i.", reason);

	/* [Legacy code] v4.01-0: There was return here before. 
	 * This became wrong because escaped status did not stop correction, 
	 * and therefore did not reset the zap counter.
	 */
	if (correction_warn_on) {
		/** @todo Notify messaging module and server. Reference old code:
		 * SetStatusReason(reason);
		 * const gps_last_fix_struct_t *gpsLastFixStruct = GPS_last_fix();
		 * NofenceMessage
		 * 	msg = { .which_m =
		 * 			NofenceMessage_client_warning_message_tag,
		 * 		.m.client_warning_message = {
		 * 			.xDatePos = proto_getLastKnownDatePos(
		 * 				gpsLastFixStruct),
		 * 			.usDuration = WarnDuration(),
		 * 			.has_sFenceDist = true,
		 * 			.sFenceDist =
		 * 				gpsp_get_inst_dist_to_border() } };
		 * log_WriteNofenceMessage(&msg);
		 */
	}

	switch (reason) {
	case Reason_WARNPAUSEREASON_COMPASS:
	case Reason_WARNPAUSEREASON_ACC:
	case Reason_WARNPAUSEREASON_MOVEBACK:
	case Reason_WARNPAUSEREASON_MOVEBACKNODIST:
	case Reason_WARNPAUSEREASON_NODIST:
		dist_add = (int16_t)_LAST_DIST_ADD;
		break;
	case Reason_WARNPAUSEREASON_BADFIX:
		dist_add = DIST_OFFSET_AFTER_BADFIX;
		break;
	case Reason_WARNPAUSEREASON_MISSGPSDATA:
		dist_add = 0;
		break;
	case Reason_WARNPAUSEREASON_ZAP:
		dist_add = DIST_OFFSET_AFTER_PAIN;
		break;
	case Reason_WARNSTOPREASON_INSIDE:
	case Reason_WARNSTOPREASON_MOVEBACKINSIDE:
	case Reason_WARNSTOPREASON_ESCAPED:
	case Reason_WARNSTOPREASON_MODE:
		dist_add = 1;
		break;
	default:
		dist_add = 0;
		break;
	}

	if (dist_add > 0) {
		/* Makes sure that sound starts over. */
		atomic_set(&last_warn_freq, WARN_FREQ_INIT);

		/* If Distance is set, then restart further warning 
		 * mentioned distance from this distance.
		 */
		last_warn_dist = mean_dist + dist_add;
		if (last_warn_dist < LIM_WARN_MIN_DM) {
			last_warn_dist = LIM_WARN_MIN_DM;
		}
	}
	correction_pause_timestamp = k_uptime_get_32();
	correction_warn_on = 0;
	k_sem_take(&correction_warn_sem, K_SECONDS(1));

	if (reason > Reason_WARNSTOPREASON) {
		correction_end();
	}
}

static void correction(Mode amc_mode, int16_t mean_dist, int16_t dist_change)
{
	/* Variables used in the correction setup, calculation and end. */
	if (zap_eval_doing) {
		uint32_t delta_zap_eval = k_uptime_get_32() - zap_timestamp;

		/** @note the zap eval time is set to 200ms, but this function
		 *  is only called 1000ms/4hz = 250ms due to every GNSS
		 *  measurement.
		 */
		if (delta_zap_eval >= ZAP_EVALUATION_TIME) {
			/** @todo Notify server and log, i.e submit event to 
			 *  the messaging module. Old code ref:
			 * 
			 * NofenceMessage msg = {
			 * 	.which_m = NofenceMessage_client_zap_message_tag,
			 * 	.m.client_zap_message = {
			 * 		.xDatePos = proto_getLastKnownDatePos(gpsLastFix),
			 * 		.has_sFenceDist = true,
			 * 		.sFenceDist = gpsp_get_inst_dist_to_border()
			 * 	}
			 * };
            		 * log_WriteNofenceMessage(&msg);
			 * \* Makes sure the updated data is 
			 *  * transferred immediately after every zap.
			 *  *\
			 * gsm_SetStatusToConnectAndTransfer(GSM_CALL_CORRECTIONZAP, false);
			 */
			zap_eval_doing = false;
		}
		return;
	} else if (correction_started) {
		if (correction_warn_on) {
			uint16_t freq = atomic_get(&last_warn_freq);
			if (freq == 0) {
				/** @todo last_warn_freq not yet set from
				 * correction_freq_thread??? Do something??
				 */
			}

			if (freq < WARN_FREQ_INIT) {
				freq = WARN_FREQ_INIT;
			}

			bool try_zap = false;

			/* Clamp max frequency so we know sound controller
			 * plays the exact MAX frequency in order to
			 * also publish an event of MAX.
			 */
			if (freq >= WARN_FREQ_MAX) {
				freq = WARN_FREQ_MAX;
				try_zap = true;
			}

			/* We check if we're within valid frequency ranges. */
			if (freq >= WARN_FREQ_INIT && freq <= WARN_FREQ_MAX) {
				/** Update buzzer frequency event. */
				struct sound_set_warn_freq_event *freq_ev =
					new_sound_set_warn_freq_event();
				freq_ev->freq = freq;
				EVENT_SUBMIT(freq_ev);

				/** @note It will zap immediately once we 
				 *  reach WARN_FREQ_MAX, and wait 200ms 
				 *  (ZAP_EVALUATION_TIME) until next zap
				 *  up to 3 times untill it is considered
				 *  "escaped."
				 */

				if (try_zap && atomic_get(&sound_max_atomic)) {
					correction_pause(
						Reason_WARNPAUSEREASON_ZAP,
						mean_dist);
					struct ep_status_event *ep_ev =
						new_ep_status_event();
					ep_ev->ep_status = EP_RELEASE;
					EVENT_SUBMIT(ep_ev);
					zap_eval_doing = true;
					zap_timestamp = k_uptime_get_32();
					increment_zap_count();
					LOG_INF("AMC notified EP to zap!");
				}
			}
		} else {
			last_warn_dist = mean_dist + _LAST_DIST_ADD;
		}
	}
}

uint8_t get_correction_status(void)
{
	return correction_started + correction_warn_on;
}

void process_correction(Mode amc_mode, gnss_last_fix_struct_t *gnss,
			FenceStatus fs, amc_zone_t zone, int16_t mean_dist,
			int16_t dist_change)
{
	if (amc_mode == Mode_Teach || amc_mode == Mode_Fence) {
		if (zone == WARN_ZONE) {
			if (gnss->mode == GNSSMODE_MAX) {
				if (fs == FenceStatus_FenceStatus_Normal ||
				    fs == FenceStatus_MaybeOutOfFence) {
					/** @todo Fetch getActiveTime() from
					 *  movement controller. What to check for?
					 *
					|| get_correction_status() > 0) { */
					if (gnss_has_warn_fix()) {
						correction_start(mean_dist);
					}
					//}
				}
			}
		}
	}
	/* Update distance atomics. Take this semaphore twice to ensure that
	 * we have calculated the correction_freq once
	 * before we process the new distances. (Only if correction started)
	 */
	atomic_set(&dist_change_atomic, dist_change);
	if (get_correction_status() > 0) {
		k_sem_take(&correction_freq_updated_sem, K_SECONDS(1));
		k_sem_take(&correction_freq_updated_sem, K_SECONDS(1));
	}

	/* Start main correction procedure. */
	correction(amc_mode, mean_dist, dist_change);

	uint32_t delta_gnss_fix = k_uptime_get_32() - gnss->updated_at;

	/* Checks for pausing correction only if we have started it. */
	if (correction_started || correction_warn_on) {
		if ((amc_mode != Mode_Teach && amc_mode != Mode_Fence) ||
		    zone == NO_ZONE) {
			correction_pause(Reason_WARNSTOPREASON_MODE, mean_dist);
		} else if (fs == FenceStatus_Escaped) {
			correction_pause(Reason_WARNSTOPREASON_ESCAPED,
					 mean_dist);
		} else if (delta_gnss_fix > GNSS_1SEC) {
			/* Warning pause as result of missing GNSS. */
			correction_pause(Reason_WARNPAUSEREASON_MISSGPSDATA,
					 mean_dist);
		} else if (!gnss_has_accepted_fix()) {
			/* Warning pause as result of bad position accuracy. */
			correction_pause(Reason_WARNPAUSEREASON_BADFIX,
					 mean_dist);
		} else if (amc_mode == Mode_Fence) {
			if (mean_dist - last_warn_dist <=
			    CORRECTION_PAUSE_DIST) {
				correction_pause(Reason_WARNPAUSEREASON_NODIST,
						 mean_dist);
			}
		} else if (amc_mode == Mode_Teach) {
			/* [LEGACY] see: https://youtrack.axbit.com/youtrack/issue/NOF-307
		 * 		if (acc_RawAmplitude(ACC_Y) > ACC_STOP_AMPLITUDE){			// This makes it easier for the animal to understand that it is in control and that it acctually is possible to turn off the warning
		 * 			correction_pause(Reason_WARNPAUSEREASON_ACC);			// Warning pause as result of that the accelerometer values shows sound reaction
		 * 		}
		 */
			if (mean_dist - last_warn_dist <=
			    TEACHMODE_CORRECTION_PAUSE_DIST) {
				correction_pause(Reason_WARNPAUSEREASON_NODIST,
						 mean_dist);
			}
			if (dist_change <= TEACHMODE_DIST_DECR_SLOPE_OFF_LIM) {
				/* Then animal has moved back, closer to fence. */
				correction_pause(
					Reason_WARNPAUSEREASON_MOVEBACK,
					mean_dist);
			}
		}
		/* [LEGACY CODE] See http://youtrack.axbit.no/youtrack/issue/NOF-213. */
		if ((get_correction_status() < 2) && zone != WARN_ZONE) {
			/* Turn off warning only if it is already 
		 * paused when inside the pasture. 
		 */
			correction_pause(Reason_WARNSTOPREASON_INSIDE,
					 mean_dist);
		}
	}
}