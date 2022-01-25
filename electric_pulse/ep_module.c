/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include <logging/log.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include "ep_module.h"
#include "ep_event.h"

#define LOG_MODULE_NAME ep_module
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_EP_MODULE_LOG_LEVEL);

// TODO: fetch this config value from EEPROM;
#define CATTLE 0

#define PIN_HIGH 1
#define PIN_LOW 0

/* Match config values from HW_N */
#define EP_DURATION_CATTLE 1000000
#define EP_DURATION_SHEEP 500000
#define EP_ON_TIME 2 // uS
#define EP_OFF_TIME 5 // uS
#define EP_FREQ (EP_ON_TIME + EP_OFF_TIME + 3) // extra delay for loop

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

const struct device *ep_ctrl_dev;
const struct device *ep_detect_dev;

int ep_module_init(void)
{
	int err;
	ep_ctrl_dev = device_get_binding(EP_CTRL_LABEL);
	if (ep_ctrl_dev == NULL) {
		LOG_ERR("Error get device %s", log_strdup(EP_CTRL_LABEL));
		return -ENODEV;
	}

	err = gpio_pin_configure(ep_ctrl_dev, EP_CTRL_PIN,
				 GPIO_OUTPUT_ACTIVE | EP_CTRL_FLAGS);

// Detect pin is not in use. Set to disconnected
#if DT_NODE_HAS_STATUS(DT_ALIAS(ep_detect), okay)
	ep_detect_dev = device_get_binding(EP_DETECT_LABEL);
	if (ep_detect_dev == NULL) {
		LOG_ERR("Error get device %s", log_strdup(EP_DETECT_LABEL));
		return -ENODEV;
	}
	err = gpio_pin_configure(ep_detect_dev, EP_DETECT_PIN,
				 GPIO_DISCONNECTED);
#endif
	if (err < 0) {
		LOG_ERR("Error initializing EP module");
		return err;
	}
	return 0;
}

static int ep_module_release(void)
{
	int err;
	uint32_t i;
	uint32_t EP_duration =
		(uint32_t)(EP_DURATION_SHEEP / EP_FREQ); // in us for sheep

	if (CATTLE) {
		EP_duration = (uint32_t)(EP_DURATION_CATTLE /
					 EP_FREQ); // in us for cattle
	}

	if (!device_is_ready(ep_ctrl_dev)) {
		/* Not ready, do not use */
		LOG_ERR("EP not ready or proper initialized");
		return -ENODEV;
	}

	for (i = 0; i < EP_duration; i++) {
		err = gpio_pin_set(ep_ctrl_dev, EP_CTRL_PIN, PIN_HIGH);
		k_busy_wait(EP_ON_TIME);
		err = gpio_pin_set(ep_ctrl_dev, EP_CTRL_PIN, PIN_LOW);
		k_busy_wait(EP_OFF_TIME);
	}

	err = gpio_pin_set(ep_ctrl_dev, EP_CTRL_PIN, PIN_LOW);
	return err;
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
		const struct ep_status_event *event = cast_ep_status_event(eh);

		switch (event->ep_status) {
		case EP_RELEASE:
			ep_module_release();
			break;
		default:
			/* Unhandled control message */
			__ASSERT_NO_MSG(false);
			break;
		}

		return false;
	}

	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, ep_status_event);
