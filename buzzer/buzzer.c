/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include <logging/log.h>
#include "sound_event.h"
#include <device.h>

#define MODULE buzzer
LOG_MODULE_REGISTER(MODULE, CONFIG_BUZZER_LOG_LEVEL);

#if DT_NODE_HAS_STATUS(DT_ALIAS(pwm_buzzer), okay)
#define BUZZER_PWM_NODE_ID DT_ALIAS(pwm_buzzer)
#define BUZZER_PWM_DEV_NAME DEVICE_DT_NAME(BUZZER_PWM_NODE_ID)
#else
#error "No BUZZER PWM device found"
#endif

const struct device *buzzer_pwm;

int buzzer_module_init(void)
{
	buzzer_pwm = device_get_binding(BUZZER_PWM_DEV_NAME);
	if (buzzer_pwm) {
		LOG_INF("Found device %s", BUZZER_PWM_DEV_NAME);
	} else {
		LOG_ERR("Device %s not found", BUZZER_PWM_DEV_NAME);
		return -ENODEV;
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
		LOG_INF("Sound event %i", ev->type);
		return false;
	}

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, sound_event);