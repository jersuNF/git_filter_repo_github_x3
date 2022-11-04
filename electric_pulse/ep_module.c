/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>
#include <logging/log.h>

#include "ep_module.h"
#include "ep_event.h"
#include "error_event.h"
#include "messaging_module_events.h"


#define MODULE ep_module
LOG_MODULE_REGISTER(MODULE, CONFIG_EP_MODULE_LOG_LEVEL);

#define PIN_HIGH 1
#define PIN_LOW 0

#define PRODUCT_TYPE_SHEEP 1
#define PRODUCT_TYPE_CATTLE 2

/* Electric pulse PWM configuration, NB! Must be fine tuned for new HW */
#define EP_DURATION_CATTLE_US_FIRST_PULSE_US 233333
#define EP_DURATION_CATTLE_US 700000

#define EP_DURATION_SHEEP_US_FIRST_PULSE_US 116666
#define EP_DURATION_SHEEP_US 350000

#define EP_ON_TIME_US 3
#define EP_OFF_TIME_US 8

/* Safety timer */
#define MINIMUM_TIME_BETWEEN_PULSES_MS 5000

/* Board with no supported hw, use LED as indicator */
#if DT_NODE_HAS_STATUS(DT_ALIAS(led0), okay)
#define EP_CTRL_NODE DT_ALIAS(led0)
#define EP_CTRL_LABEL DT_GPIO_LABEL(EP_CTRL_NODE, gpios)
#define EP_CTRL_PIN DT_GPIO_PIN(EP_CTRL_NODE, gpios)
#define EP_CTRL_FLAGS DT_GPIO_FLAGS(EP_CTRL_NODE, gpios)
#else
#define EP_CTRL_NODE DT_ALIAS(ep_ctrl)
#define EP_CTRL_LABEL DT_GPIO_LABEL(EP_CTRL_NODE, gpios)
#define EP_CTRL_PIN DT_GPIO_PIN(EP_CTRL_NODE, gpios)
#define EP_CTRL_FLAGS DT_GPIO_FLAGS(EP_CTRL_NODE, gpios)

#define EP_DETECT_NODE DT_ALIAS(ep_detect)
#define EP_DETECT_LABEL DT_GPIO_LABEL(EP_DETECT_NODE, gpios)
#define EP_DETECT_PIN DT_GPIO_PIN(EP_DETECT_NODE, gpios)
#define EP_DETECT_FLAGS DT_GPIO_FLAGS(EP_DETECT_NODE, gpios)
#endif

#if DT_NODE_HAS_STATUS(DT_ALIAS(pwm_ep), okay)
#define PWM_EP_NODE DT_ALIAS(pwm_ep)
#define PWM_EP_LABEL DT_GPIO_LABEL(PWM_EP_NODE, pwms)
#define PWM_EP_CHANNEL DT_PWMS_CHANNEL(DT_ALIAS(pwm_ep))
#endif

const struct device *ep_ctrl_dev;
const struct device *ep_ctrl_pwm_dev;
const struct device *ep_detect_dev;
static uint16_t g_product_type = 0;
static int64_t g_last_pulse_time = 0;
volatile bool g_trigger_ready = false;
extern struct k_sem ep_trigger_ready;

int ep_module_init(void)
{
	int ret;
	do {
		ep_ctrl_dev = device_get_binding(EP_CTRL_LABEL);
		if (ep_ctrl_dev == NULL) {
			LOG_WRN("Error get device %s", log_strdup(EP_CTRL_LABEL));
			ret = -ENODEV;
			break;
		}
		ret = gpio_pin_configure(ep_ctrl_dev, EP_CTRL_PIN, (GPIO_OUTPUT_ACTIVE | 
			EP_CTRL_FLAGS | GPIO_DS_ALT_HIGH  | GPIO_DS_ALT_LOW));
		if (ret != 0) {
			LOG_WRN("Failed to configure EP control pin");
			break;
		}
		ret = gpio_pin_set(ep_ctrl_dev, EP_CTRL_PIN, PIN_LOW);
		if (ret != 0) {
			LOG_WRN("Failed to set initial value of EP control pin");
			break;
		}

		ep_ctrl_pwm_dev = device_get_binding(PWM_EP_LABEL);
		if (ep_ctrl_pwm_dev == NULL) {
			LOG_WRN("Failed to get EP PWM device");
			ret = -ENODEV;
			break;
		}

		ret = stg_config_u16_read(STG_U16_PRODUCT_TYPE, &g_product_type);
		if (ret != 0) {
			LOG_WRN("Failed to read product type from flash");
			break;
		}
		if ((g_product_type != PRODUCT_TYPE_SHEEP) && (g_product_type != PRODUCT_TYPE_CATTLE)) {
			LOG_WRN("Unknown product type, EP module set to sheep configuration");
			g_product_type = PRODUCT_TYPE_SHEEP;
		}
		return 0;
	}while(0);
	LOG_ERR("Failed to initializing EP module!");
	return ret;
}

static int ep_module_release(bool first_pulse)
{
	if (!device_is_ready(ep_ctrl_pwm_dev)) {
		LOG_WRN("Electic pulse PWM device not ready!");
		return -ENODEV;
	}
	if (!device_is_ready(ep_ctrl_dev)) {
		LOG_WRN("Electic pulse GPIO device not ready!");
		return -ENODEV;
	}

	int64_t current_time = k_uptime_get();
	int64_t elapsed_time = current_time - g_last_pulse_time;
	if (elapsed_time < MINIMUM_TIME_BETWEEN_PULSES_MS) {
		LOG_WRN("Time between EP is shorter than allowed");
		return -EACCES;
	}

	uint32_t ep_duration_us = (uint32_t)EP_DURATION_SHEEP_US;
	if (g_product_type == PRODUCT_TYPE_CATTLE) {
		if (first_pulse) {
			ep_duration_us = (uint32_t)EP_DURATION_CATTLE_US_FIRST_PULSE_US;
		} else {
			ep_duration_us = (uint32_t)EP_DURATION_CATTLE_US;
		}
	} else {
		if (first_pulse) {
			ep_duration_us =
				(uint32_t)EP_DURATION_SHEEP_US_FIRST_PULSE_US;
		} else {
			ep_duration_us = (uint32_t)EP_DURATION_SHEEP_US;
		}
	}

	LOG_INF("Triggering electric pulse now (Period[us]:%d, Pulse width[us]:%d, Duration[us]:%d)", 
			(EP_ON_TIME_US + EP_OFF_TIME_US), 
			EP_ON_TIME_US, 
			ep_duration_us);


	int ret;
	/* Turn ON electric pulse signal (PWM) */
	ret = pwm_pin_set_usec(ep_ctrl_pwm_dev, PWM_EP_CHANNEL, 
							(EP_ON_TIME_US + EP_OFF_TIME_US), EP_ON_TIME_US, 0);
	if (ret != 0) {
		LOG_WRN("Unable to set electic pulse PWM signal!");
	}
	else {
		k_busy_wait(ep_duration_us);
		g_last_pulse_time = current_time;
	}
		
	/* Turn OFF electric pulse signal (PWM) */
	ret = pwm_pin_set_usec(ep_ctrl_pwm_dev, PWM_EP_CHANNEL, 0, 0, 0);
	if (ret != 0) { 
		LOG_WRN("Unable to disable electic pulse PWM signal!"); 
	}
	ret = gpio_pin_set(ep_ctrl_dev, EP_CTRL_PIN, PIN_LOW);
	if (ret != 0) {
		LOG_WRN("Unable to disable electic pulse GPIO signal!");
	}

	return ret;
}

/** 
 * @brief Event handler function
 * @param[in] eh Pointer to event handler struct
 * @return true to consume the event (event is not propagated to further
 * listners), false otherwise
 */
static bool event_handler(const struct event_header *eh)
{
	/* Received ep status event */
	if (is_ep_status_event(eh)) {
		int err;
		const struct ep_status_event *event = cast_ep_status_event(eh);
		switch (event->ep_status) {
			case EP_RELEASE: {
				if (g_trigger_ready) {
					g_trigger_ready = false;
					err = ep_module_release(event->is_first_pulse);
					if (err < 0) {
						char *e_msg = "Error in ep release";
						LOG_ERR("%s (%d)", log_strdup(e_msg), err);
						nf_app_error(ERR_EP_MODULE, err, e_msg, strlen(e_msg));
					}
				} else {
					char *e_msg = "Tried to give EP outside sound max event";
					LOG_ERR("%s (%d)", log_strdup(e_msg), -EACCES);
					nf_app_error(ERR_EP_MODULE, -EACCES, e_msg, strlen(e_msg));
				}
				break;
			}
			default: {
				/* Unhandled control message */
				__ASSERT_NO_MSG(false);
				break;
			}
		}
		return false;
	}
	if (is_warn_correction_pause_event(eh)) {
		/* Open up window for zapping since we recieved that
		 * the sound controller is playing MAX warn freq. */
		const struct warn_correction_pause_event *event
			= cast_warn_correction_pause_event(eh);
		if (event->reason == Reason_WARNPAUSEREASON_ZAP) {
			g_trigger_ready = true;
			k_sem_give(&ep_trigger_ready);
		} else {
			g_trigger_ready = false;
		}
		return false;
	}
	__ASSERT_NO_MSG(false);
	return false;
}

EVENT_LISTENER(LOG_MODULE_NAME, event_handler);
EVENT_SUBSCRIBE(LOG_MODULE_NAME, ep_status_event);
EVENT_SUBSCRIBE_EARLY(LOG_MODULE_NAME, warn_correction_pause_event);
