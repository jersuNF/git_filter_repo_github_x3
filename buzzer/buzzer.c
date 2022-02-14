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

K_MSGQ_DEFINE(msgq_sound_events, sizeof(struct sound_event),
	      CONFIG_MSGQ_SOUND_EVENT_SIZE, 4);

const struct device *buzzer_pwm;

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
void set_pwm_to_idle(void)
{
	int err = pwm_pin_set_usec(buzzer_pwm, PWM_CHANNEL, 0, 0, 0);
	if (err) {
		LOG_ERR("pwm off fails %i", err);
		return;
	}
}

void play_note(note_t note, bool pwm_off_after_played)
{
	int err = pwm_pin_set_usec(buzzer_pwm, PWM_CHANNEL, note.t, note.t / 2U,
				   0);
	if (err) {
		LOG_ERR("Error %d: failed to set pulse width", err);
		return;
	}

	/* Play note for 'sustain (s)' milliseconds. */
	k_sleep(K_MSEC(note.s));

	if (pwm_off_after_played) {
		/* Fix powermode for this function. */
		set_pwm_to_idle();
	}
}

void play_song(const note_t *song, const size_t num_notes)
{
	/* Play all the notes in the note array. Do not turn of PWM
	 * between notes as that is just unecessary when we change the note
	 * straight after. Turn off PWM (set to 0) when song is finished.
	 */
	for (int i = 0; i < num_notes; i++) {
		play_note(song[i], false);
	}

	/* Fix powermode for this function. */
	set_pwm_to_idle();
}

/* Work out a priority sound system. Solve multiple event submissions. */
void play_type(enum sound_event_type type)
{
	LOG_INF("Received sound event %d", type);
	if (buzzer_pwm == NULL) {
		LOG_ERR("Buzzer PWM not yet initialized.");
		return;
	}
	switch (type) {
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
		while (k_msgq_put(&msgq_sound_events, ev, K_NO_WAIT) != 0) {
			k_msgq_purge(&msgq_sound_events);
		}

		return false;
	}

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, sound_event);

void buzzer_thread_fn()
{
	while (true) {
		struct sound_event ev;
		int err = k_msgq_get(&msgq_sound_events, &ev, K_FOREVER);
		if (err) {
			LOG_ERR("Error getting sound_event: %d", err);
			return;
		}
		play_type(ev.type);
	}
}

K_THREAD_DEFINE(buzzer_thread, CONFIG_BUZZER_THREAD_SIZE, buzzer_thread_fn,
		NULL, NULL, NULL, CONFIG_BUZZER_THREAD_PRIORITY, 0, 0);