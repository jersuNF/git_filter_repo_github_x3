/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include <logging/log.h>
#include "sound_event.h"
#include "buzzer.h"
#include <device.h>
#include <drivers/pwm.h>

#define MODULE buzzer
LOG_MODULE_REGISTER(MODULE, CONFIG_BUZZER_LOG_LEVEL);

#if DT_NODE_HAS_STATUS(DT_ALIAS(pwm_buzzer), okay)
#define PWM_BUZZER_NODE DT_ALIAS(pwm_buzzer)
#define PWM_BUZZER_LABEL DT_GPIO_LABEL(PWM_BUZZER_NODE, pwms)
#define PWM_CHANNEL DT_PWMS_CHANNEL(DT_ALIAS(pwm_buzzer))
#else
#error "Choose a supported PWM driver"
#endif

const struct device *buzzer_pwm;

static volatile enum sound_event_type current_type = SND_READY_FOR_NEXT_TYPE;
atomic_t stop_current_sound = ATOMIC_INIT(false);
static struct k_work_q sound_q;
static struct k_work sound_work;
K_THREAD_STACK_DEFINE(sound_buzzer_area, CONFIG_BUZZER_THREAD_SIZE);

static const note_t m_geiterams[] = {
	{ .t = tone_nothing, .s = d_16 },
	{ .t = tone_e_6, .s = d_16 },
	{ .t = tone_e_6, .s = d_16 },
	{ .t = tone_e_6, .s = d_16 },
	{ .t = tone_d_6, .s = d_16 },
	{ .t = tone_c_6, .s = d_8 },
	{ .t = tone_c_6, .s = d_8 },
	/* .. */
	{ .t = tone_e_6, .s = d_16 },
	{ .t = tone_g_6, .s = d_16 },
	{ .t = tone_c_7, .s = d_16 },
	{ .t = tone_e_6, .s = d_16 },
	{ .t = tone_g_6, .s = d_8 },
	{ .t = tone_g_6, .s = d_8 },
	/* .. */
	{ .t = tone_b_6, .s = d_16 },
	{ .t = tone_b_6, .s = d_16 },
	{ .t = tone_b_6, .s = d_16 },
	{ .t = tone_a_6, .s = d_16 },
	{ .t = tone_f_6, .s = d_8 },
	{ .t = tone_f_6, .s = d_8 },
	/* .. */
	{ .t = tone_a_6, .s = d_16 },
	{ .t = tone_a_6, .s = d_16 },
	{ .t = tone_a_6, .s = d_16 },
	{ .t = tone_g_6, .s = d_16 },
	{ .t = tone_e_6, .s = d_8 },
	{ .t = tone_e_6, .s = d_8 },
};

static const note_t m_perspelmann[] = {
	{ .t = tone_nothing, .s = d_16 },
	{ .t = tone_c_7, .s = d_16 },
	{ .t = tone_c_7, .s = d_8 },
	{ .t = tone_g_6, .s = d_8 },
	{ .t = tone_aiss_6, .s = d_16 },
	{ .t = tone_a_6, .s = d_8 },
	{ .t = tone_f_6, .s = d_8 },
	{ .t = tone_f_7, .s = d_16 },
	{ .t = tone_e_6, .s = d_8 },
	/* .. */
	{ .t = tone_f_6, .s = d_8 },
	{ .t = tone_d_6, .s = d_16 },
	{ .t = tone_c_6, .s = d_8 },
};

#define n_geiterams_notes (sizeof(m_geiterams) / sizeof(m_geiterams[0]))
#define n_perspelmann_notes (sizeof(m_perspelmann) / sizeof(m_perspelmann[0]))

/* Fix powermode for this function. */
int set_pwm_to_idle(void)
{
	int err = pwm_pin_set_usec(buzzer_pwm, PWM_CHANNEL, 0, 0, 0);
	if (err) {
		LOG_ERR("pwm off fails %i", err);
		return err;
	}
	return 0;
}

/** @brief Plays a note.
 * 
 * @param note note to be played, contains freq and sustain
 * 
 * @param pwm_off_after_played sets the pwm off if true
 * 
 * @return true if ready for next note, false if we want to terminate sequence
 */
int play_note(note_t note, bool pwm_off_after_played)
{
	/* Check variable set by event_handler thread when we
	 * receive new events with higher priority, terminate if true.
	 */
	if (atomic_get(&stop_current_sound)) {
		return false;
	}

	int err = pwm_pin_set_usec(buzzer_pwm, PWM_CHANNEL, note.t, note.t / 2U,
				   0);
	if (err) {
		LOG_ERR("Error %d: failed to set pulse width", err);
		return false;
	}

	/* Play note for 'sustain (s)' milliseconds. */
	k_sleep(K_MSEC(note.s));

	if (pwm_off_after_played) {
		/* Fix powermode for this function. */
		err = set_pwm_to_idle();
		if (err) {
			return false;
		}
	}
	return true;
}

void play_song(const note_t *song, const size_t num_notes)
{
	/* Play all the notes in the note array. Do not turn of PWM
	 * between notes as that is just unecessary when we change the note
	 * straight after. Turn off PWM (set to 0) when song is finished.
	 */
	for (int i = 0; i < num_notes; i++) {
		if (!play_note(song[i], false)) {
			/* Exit if function returns false, this means
			 * the atomic variable set by event handler
			 * is true and we have a higher priority sound
			 * incomming.
			 */
			return;
		}
	}

	/* Fix powermode for this function. */
	set_pwm_to_idle();
}

void play()
{
	if (buzzer_pwm == NULL) {
		LOG_ERR("Buzzer PWM not yet initialized.");
		return;
	}

	/* Starting a new sound type, set atomic to false. */
	atomic_set(&stop_current_sound, false);

	switch (current_type) {
	case SND_WELCOME: {
		note_t c6 = { .t = tone_c_6, .s = d_8 };
		play_note(c6, true);
		break;
	}
	case SND_PERSPELMANN: {
		play_song(m_perspelmann, n_perspelmann_notes);
		break;
	}
	case SND_FIND_ME: {
		play_song(m_geiterams, n_geiterams_notes);
		break;
	}
	default: {
		break;
	}
	}
	current_type = SND_READY_FOR_NEXT_TYPE;
}

int buzzer_module_init(void)
{
	uint64_t cycles;
	buzzer_pwm = device_get_binding(PWM_BUZZER_LABEL);
	if (!buzzer_pwm) {
		LOG_ERR("Cannot find buzzer PWM device!");
		return -ENODEV;
	}

	int err = pwm_get_cycles_per_sec(buzzer_pwm, PWM_CHANNEL, &cycles);
	if (err) {
		LOG_ERR("Error getting clock cycles for PWM %d", err);
		return err;
	}

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

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, sound_event);

void init_sound_controller(void)
{
	k_work_queue_init(&sound_q);
	k_work_queue_start(&sound_q, sound_buzzer_area,
			   K_THREAD_STACK_SIZEOF(sound_buzzer_area),
			   CONFIG_BUZZER_THREAD_PRIORITY, NULL);
	k_work_init(&sound_work, play);
}