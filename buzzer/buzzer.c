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

void play_sound(int freq, int dur)
{
	int err = pwm_pin_set_usec(buzzer_pwm, PWM_CHANNEL, freq, freq / 2U, 0);
	if (err) {
		LOG_ERR("Error %d: failed to set pulse width", err);
		return;
	}

	k_sleep(K_SECONDS(dur));

	// Disable
	err = pwm_pin_set_usec(buzzer_pwm, PWM_CHANNEL, 0, 0, 0);
	if (err) {
		LOG_ERR("pwm off fails %i", err);
		return;
	}
	LOG_INF("Disabled buzzer again.");
}

void play_type(enum sound_event_type type)
{
	LOG_INF("Received sound event %d", type);
	if (buzzer_pwm == NULL) {
		LOG_ERR("Buzzer PWM not yet initialized.");
		return;
	}
	switch (type) {
	case SND_WELCOME: {
		play_sound(2273, 5);
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
		struct sound_event *ev;
		int err = k_msgq_get(&msgq_sound_events, &ev, K_FOREVER);
		if (err) {
			LOG_ERR("Error getting sound_event: %d", err);
			return;
		}
		if (ev == NULL) {
			LOG_ERR("No sound event available.");
			continue;
		}
		play_type(ev->type);
	}
}

K_THREAD_DEFINE(buzzer_thread, CONFIG_BUZZER_THREAD_SIZE, buzzer_thread_fn,
		NULL, NULL, NULL, CONFIG_BUZZER_THREAD_PRIORITY, 0, 0);