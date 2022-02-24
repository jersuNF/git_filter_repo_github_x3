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
atomic_t stop_current_sound = ATOMIC_INIT(false);
static struct k_work_q sound_q;
static struct k_work sound_work;
K_THREAD_STACK_DEFINE(sound_buzzer_area, CONFIG_BUZZER_THREAD_SIZE);

atomic_t current_warn_zone_freq = ATOMIC_INIT(0);

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
	return (freq / 2) * (volume / 100);
}

/** @brief Plays a frequency for given duration.
 * 
 * @param freq frequency tone to be played in hz.
 * @param sustain length/duration of the note/frequency to be played in ms
 * @param volume loudness of the played frequency, ranging from 0%-100%
 *               This is simply having a ratio for a duty cycle between 0%-50%.
 * 
 * @return 0 on success, otherwise negative errno.
 */
int play_freq(const uint32_t freq, const uint32_t sustain, const uint8_t volume)
{
	int err;

	/* Check variable set by event_handler thread when we
	 * receive new events with higher priority, terminate if true.
	 */
	if (atomic_get(&stop_current_sound)) {
		err = -EINTR;
		goto set_to_idle;
	}

	uint32_t pulse = get_pulse_width(freq, volume);

	err = pwm_pin_set_usec(buzzer_pwm, PWM_CHANNEL, freq, pulse, 0);

	if (err) {
		LOG_ERR("Error %d: failed to set pulse width", err);
		goto set_to_idle;
	}

	/* Play note for 'sustain (s)' milliseconds. */
	k_sleep(K_MSEC(sustain));

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
		int ret = play_freq(song[i].t, song[i].s,
				    BUZZER_SOUND_VOLUME_PERCENT);
		/* Exit song if we get interrupted (new sound with lower 
		 * priority queued or error occured.)
		 */
		if (ret) {
			return;
		}
	}
}

void start_warn_zone_loop(void)
{
	///* While any higher priority sounds is not queued up. */
	//while (!atomic_get(&stop_current_sound)) {
	//	/* While frequency is higher than minimum value. */
	//	do {
	//		int freq_value = atomic_get(&stop_current_sound);
	//	} while (current_warn_zone_freq <);
	//}
}

void play()
{
	if (buzzer_pwm == NULL) {
		LOG_ERR("Buzzer PWM not yet initialized.");
		return;
	}

	/* Starting a new sound type, set atomic to false. */
	atomic_set(&stop_current_sound, false);

	struct sound_status_event *ev_playing = new_sound_status_event();

	switch (current_type) {
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
	case SND_MAX: {
		ev_playing->status = SND_STATUS_PLAYING_MAX;
		EVENT_SUBMIT(ev_playing);

		/* Play max freq here. */
		break;
	}
	case SND_WARN: {
		ev_playing->status = SND_WARN;
		EVENT_SUBMIT(ev_playing);

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
		 * with atomic variable whether it should stop playing or
		 * continue.
		 */
		if (ev->type > current_type) {
			return false;
		} else {
			atomic_set(&stop_current_sound, true);
			current_type = ev->type;
			k_work_submit_to_queue(&sound_q, &sound_work);
		}
		return false;
	}
	if (is_sound_set_warn_freq_event(eh)) {
		struct sound_set_warn_freq_event *ev =
			cast_sound_set_warn_freq_event(eh);
		atomic_set(&current_warn_zone_freq, ev->freq);

		return false;
	}

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, sound_event);
EVENT_SUBSCRIBE(MODULE, sound_set_warn_freq_event);