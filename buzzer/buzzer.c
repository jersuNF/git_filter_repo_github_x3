/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include <logging/log.h>
#include "sound_event.h"
#include "buzzer.h"
#include <device.h>
#include <drivers/pwm.h>
#include "melodies.h"

#include "error_event.h"

#define MODULE buzzer
LOG_MODULE_REGISTER(MODULE, CONFIG_BUZZER_LOG_LEVEL);

#if DT_NODE_HAS_STATUS(DT_ALIAS(pwm_buzzer), okay)
#define PWM_BUZZER_NODE DT_ALIAS(pwm_buzzer)
#define PWM_BUZZER_LABEL DT_GPIO_LABEL(PWM_BUZZER_NODE, pwms)
#define PWM_CHANNEL DT_PWMS_CHANNEL(DT_ALIAS(pwm_buzzer))
#endif

#define BUZZER_SOUND_VOLUME_PERCENT 100

const struct device *buzzer_pwm;

static volatile enum sound_event_type current_type = SND_READY_FOR_NEXT_TYPE;
K_SEM_DEFINE(stop_sound_sem, 1, 1);
static volatile bool stop_current_sound = false;
static struct k_work stop_sound_work;

static struct k_work_q sound_q;
static struct k_work sound_work;
K_THREAD_STACK_DEFINE(sound_buzzer_area, CONFIG_BUZZER_THREAD_SIZE);

void warn_zone_timeout_handler(struct k_timer *dummy);
K_TIMER_DEFINE(warn_zone_timeout_timer, warn_zone_timeout_handler, NULL);

K_SEM_DEFINE(warn_zone_sem, 1, 1);
uint32_t current_warn_zone_freq = 0;

/* Check power consumption for this PWM set. */
int set_pwm_to_idle(void)
{
	int err = pwm_pin_set_usec(buzzer_pwm, PWM_CHANNEL, 0, 0, 0);
	if (err) {
		LOG_ERR("pwm off fails %i", err);
		return err;
	}
	return 0;
}

/** @brief Gives the early sound exit sem which means
*          that if we're currently waiting for the timeout
*          in the sound thread during play_freq, we exit early so
*          we can play the next sound immediately.
*/
static void inline end_current_sound(void)
{
	k_tid_t s_thread = k_work_queue_thread_get(&sound_q);
	k_wakeup(s_thread);
}

/** @brief Calculates the necessary duty cycle to receive the wanted volume
 *         level percent given and gives pulse width.
 * 
 * @param volume loudness of the played frequency, ranging from 0-100%
 *               This is simply having a ratio for a duty cycle between 0%-50%.
 * 
 * @return pulse width to output target loudness level.
 */
static inline uint32_t get_pulse_width(uint32_t freq, uint8_t volume)
{
	return (freq / 2) * ((float)volume / 100);
}

/** @brief Plays a frequency for given duration.
 * 
 * @param freq frequency tone to be played in hz.
 * @param sustain length/duration of the note/frequency to be played in ms
 * @param volume loudness of the played frequency, ranging from 0%-100%
 *               This is simply having a ratio for a duty cycle between 0%-50%.
 * 
 * @return 0 on success, otherwise negative errno.
 * @return -EINTR if sound was aborted by another thread.
 */
int play_freq(const uint32_t freq, const uint32_t sustain, const uint8_t volume)
{
	int err;

	uint32_t pulse = get_pulse_width(freq, volume);

	err = pwm_pin_set_usec(buzzer_pwm, PWM_CHANNEL, freq, pulse, 0);
	if (err) {
		LOG_ERR("Error %d: failed to set pulse width", err);
		goto set_to_idle;
	}

	/* Clamp sustain to config to prevent semaphores to timeout
	 * during the note being played.
	 */
	uint32_t duration =
		sustain > MSEC_PER_SEC * CONFIG_BUZZER_LONGEST_NOTE_SUSTAIN ?
			(MSEC_PER_SEC * CONFIG_BUZZER_LONGEST_NOTE_SUSTAIN) -
				1 :
			sustain;

	int ticks = k_sleep(K_MSEC(duration));
	if (ticks > 0) {
		err = -EINTR;
		goto set_to_idle;
	}

set_to_idle : {
	int pwm_idle_err = set_pwm_to_idle();
	if (pwm_idle_err) {
		return pwm_idle_err;
	}
	return err;
}
}

void play_song(const note_t *song, const size_t num_notes)
{
	for (int i = 0; i < num_notes; i++) {
		/* Exit if we get the command to stop playing. */
		int err = k_sem_take(
			&stop_sound_sem,
			K_SECONDS(CONFIG_BUZZER_LONGEST_NOTE_SUSTAIN));
		if (err || stop_current_sound) {
			k_sem_give(&stop_sound_sem);
			return;
		}
		k_sem_give(&stop_sound_sem);

		err = play_freq(song[i].t, song[i].s,
				BUZZER_SOUND_VOLUME_PERCENT);
		if (err) {
			/* We can process -EINTR if we want here. */
			return;
		}
	}
}

/** @brief Timeout function that is called if we do not get a new updated
 *         warn zone frequency within that duration, ultimately ending
 *         the warn zone event.
 */
void warn_zone_timeout_handler(struct k_timer *dummy)
{
	ARG_UNUSED(dummy);

	/* Stop the current sound being played. */
	k_work_submit(&stop_sound_work);

	/* Warning here. */
	char *msg = "Timeout on getting a new warn zone freq";
	nf_app_warning(ERR_SENDER_SOUND_CONTROLLER, -ETIMEDOUT, msg,
		       strlen(msg));
}

/** @brief Starts the while loop for playing warn zone. The conditions that
 *         will break the loop is either that another sound event has been queued
 *         that has higher priority, or the current_warn_zone_freq is outside
 *         the min/max range (i.e animal is not in warn zone anymore.)
 * 
 * @note It's is required to set the frequency before submitting the 
 *       warn zone event, otherwise the buzzer will process the warn zone event
 *       and then see that the warn_zone_frequency is 0 and stop looping.
 * 
 * @note The the frequency recevied from AMC must be exactly equal to
 *       WARN_FREQ_MS_PERIOD_MAX in order to publish the
 *       SND_STATUS_PLAYING_MAX event for the EP module to subscribe to
 *       for instance.
 */
void start_warn_zone_loop(void)
{
	int err = k_sem_take(&warn_zone_sem,
			     K_SECONDS(CONFIG_BUZZER_LONGEST_NOTE_SUSTAIN));
	if (err) {
		LOG_ERR("Timeout on waiting for warn_zone semaphore.");
		goto exit_error;
	}
	/* Send a playing event if frequency is valid and NOT max. */
	if (current_warn_zone_freq <= WARN_FREQ_MS_PERIOD_INIT &&
	    current_warn_zone_freq > WARN_FREQ_MS_PERIOD_MAX) {
		struct sound_status_event *evs = new_sound_status_event();
		evs->status = SND_STATUS_PLAYING;
		EVENT_SUBMIT(evs);
	}
	k_sem_give(&warn_zone_sem);
	while (true) {
		uint32_t warn_zone_freq_duration =
			CONFIG_BUZZER_WARN_ZONE_FREQ_INTERVAL;

		err = k_sem_take(&warn_zone_sem,
				 K_SECONDS(CONFIG_BUZZER_LONGEST_NOTE_SUSTAIN));
		if (err) {
			LOG_ERR("Timeout on waiting for warn_zone semaphore.");
			goto exit_error;
		}

		/* Exit if we get the command to stop playing. */
		err = k_sem_take(&stop_sound_sem,
				 K_SECONDS(CONFIG_BUZZER_LONGEST_NOTE_SUSTAIN));
		if (err) {
			goto exit_error;
		}
		if (stop_current_sound) {
			goto exit_stopped_sound;
		}
		k_sem_give(&stop_sound_sem);

		/* If it's out of range, continue.
		 * We break out of the loop when we timeout.
		 */
		if (current_warn_zone_freq > WARN_FREQ_MS_PERIOD_INIT ||
		    current_warn_zone_freq < WARN_FREQ_MS_PERIOD_MAX) {
			k_sem_give(&warn_zone_sem);

			/* Sleep a bit and check again if freq is updated to
			 * a valid value. 
			 */
			k_sleep(K_MSEC(50));
			continue;
		}

		/* Submits MAX event if the frequency is at max. 
		 * It is submitted BEFORE the max sound is played for n seconds
		 * meaning that i.e the EP module has n seconds window to play
		 * a zap command given from AMC as well until the sound
		 * controller submits an IDLE event again.
		 */
		if (current_warn_zone_freq == WARN_FREQ_MS_PERIOD_MAX) {
			struct sound_status_event *ev =
				new_sound_status_event();
			ev->status = SND_STATUS_PLAYING_MAX;
			EVENT_SUBMIT(ev);

			warn_zone_freq_duration =
				CONFIG_BUZZER_MAX_WARN_ZONE_FREQ_INTERVAL;
		}

		err = play_freq(current_warn_zone_freq, warn_zone_freq_duration,
				BUZZER_SOUND_VOLUME_PERCENT);
		if (err) {
			k_sem_give(&warn_zone_sem);
			if (err == -EINTR) {
				/* Next warn zone frequency available, 
				 * restart and play_freq. 
				 */
				continue;
			}
			goto exit_error;
		}
		k_sem_give(&warn_zone_sem);
	}
	/* Goto labels for easier overview of the different scenarios that
	 * can abort the warn zone playing.
	 */
exit_stopped_sound : {
	k_sem_give(&stop_sound_sem);
	k_sem_give(&warn_zone_sem);
	LOG_INF("Stopped SND_WARN due to higher priority or new frequency timeout.");
	return;
}
exit_error : {
	k_sem_give(&stop_sound_sem);
	k_sem_give(&warn_zone_sem);
	char *msg = "Error occured during warn zone playing.";
	nf_app_error(ERR_SENDER_SOUND_CONTROLLER, err, msg, strlen(msg));
	return;
}
}

void play()
{
	if (buzzer_pwm == NULL) {
		LOG_ERR("Buzzer PWM not yet initialized.");
		return;
	}

	/* Starting a new sound type, set to false. */
	int err = k_sem_take(&stop_sound_sem,
			     K_SECONDS(CONFIG_BUZZER_LONGEST_NOTE_SUSTAIN));
	if (err) {
		LOG_ERR("Timeout on stopping sound sem.");
		return;
	}
	stop_current_sound = false;
	k_sem_give(&stop_sound_sem);

	struct sound_status_event *ev_playing = new_sound_status_event();

	switch (current_type) {
	case SND_OFF: {
		err = set_pwm_to_idle();
		if (err) {
			return;
		}
		break;
	}
	case SND_WELCOME: {
		ev_playing->status = SND_STATUS_PLAYING;
		EVENT_SUBMIT(ev_playing);

		play_freq(tone_c_6, d_8, BUZZER_SOUND_VOLUME_PERCENT);
		break;
	}
	case SND_PERSPELMANN: {
		ev_playing->status = SND_STATUS_PLAYING;
		EVENT_SUBMIT(ev_playing);

		play_song(m_perspelmann, n_perspelmann_notes);
		break;
	}
	case SND_FIND_ME: {
		ev_playing->status = SND_STATUS_PLAYING;
		EVENT_SUBMIT(ev_playing);

		play_song(m_geiterams, n_geiterams_notes);
		break;
	}
	case SND_WARN: {
		start_warn_zone_loop();
		break;
	}
	default: {
		break;
	}
	}

	struct sound_status_event *ev_idle = new_sound_status_event();
	ev_idle->status = SND_STATUS_IDLE;
	EVENT_SUBMIT(ev_idle);

	current_type = SND_READY_FOR_NEXT_TYPE;
}

void stop_sound_fn(struct k_work *item)
{
	ARG_UNUSED(item);

	int err = k_sem_take(&stop_sound_sem,
			     K_SECONDS(CONFIG_BUZZER_LONGEST_NOTE_SUSTAIN));
	if (err) {
		LOG_ERR("Timeout on stopping sound sem.");
		return;
	}
	stop_current_sound = true;

	k_sem_give(&stop_sound_sem);
}

int buzzer_module_init(void)
{
	uint64_t cycles;
	buzzer_pwm = device_get_binding(PWM_BUZZER_LABEL);
	if (!buzzer_pwm) {
		LOG_ERR("Cannot find buzzer PWM device! %s",
			log_strdup(PWM_BUZZER_LABEL));
		return -ENODEV;
	}

	int err = pwm_get_cycles_per_sec(buzzer_pwm, PWM_CHANNEL, &cycles);
	if (err) {
		LOG_ERR("Error getting clock cycles for PWM %d", err);
		return err;
	}

	k_work_queue_init(&sound_q);
	k_work_queue_start(&sound_q, sound_buzzer_area,
			   K_THREAD_STACK_SIZEOF(sound_buzzer_area),
			   CONFIG_BUZZER_THREAD_PRIORITY, NULL);
	k_work_init(&sound_work, play);
	k_work_init(&stop_sound_work, stop_sound_fn);

	struct sound_status_event *ev = new_sound_status_event();
	ev->status = SND_STATUS_IDLE;
	EVENT_SUBMIT(ev);

	return 0;
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
	if (is_sound_event(eh)) {
		struct sound_event *ev = cast_sound_event(eh);

		/* Check if we want to process or not. Must be done here
		 * since sound thread is busy playing. Notify sound thread
		 * with whether it should stop playing or
		 * continue.
		 */
		if (ev->type <= current_type) {
			int err = k_sem_take(
				&stop_sound_sem,
				K_SECONDS(CONFIG_BUZZER_LONGEST_NOTE_SUSTAIN));
			if (err) {
				LOG_ERR("Timeout on stopping sound sem.");
				return false;
			}
			stop_current_sound = true;
			k_sem_give(&stop_sound_sem);
			end_current_sound();

			current_type = ev->type;
			k_work_submit_to_queue(&sound_q, &sound_work);

			if (ev->type == SND_WARN) {
				/* If current type is warn zone, start timeout timer
			 	 * for getting a new frequency to play. */
				k_timer_start(
					&warn_zone_timeout_timer,
					K_SECONDS(
						CONFIG_BUZZER_UPDATE_WARN_FREQ_TIMEOUT_SEC),
					K_NO_WAIT);
			} else {
				/* Stop any existing timeout timers if we have
				 * another event type than warn zone.
				 */
				k_timer_stop(&warn_zone_timeout_timer);
			}
		}
		return false;
	}
	if (is_sound_set_warn_freq_event(eh)) {
		struct sound_set_warn_freq_event *ev =
			cast_sound_set_warn_freq_event(eh);

		/* Restart play_freq as we have a new freq, if currently
		 * playing SND_WARN and restart timeout timer.
		 */
		if (current_type == SND_WARN) {
			end_current_sound();
			k_timer_start(
				&warn_zone_timeout_timer,
				K_SECONDS(
					CONFIG_BUZZER_UPDATE_WARN_FREQ_TIMEOUT_SEC),
				K_NO_WAIT);
		} else {
			k_timer_stop(&warn_zone_timeout_timer);
		}

		int err = k_sem_take(
			&warn_zone_sem,
			K_SECONDS(CONFIG_BUZZER_LONGEST_NOTE_SUSTAIN));
		if (err) {
			LOG_ERR("Timeout on waiting for warn_zone semaphore.");
		} else {
			current_warn_zone_freq = ev->freq;
		}
		k_sem_give(&warn_zone_sem);
		return false;
	}

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, sound_event);
EVENT_SUBSCRIBE_EARLY(MODULE, sound_set_warn_freq_event);