/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include <logging/log.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include "ep_module.h"

#define LOG_MODULE_NAME ep_module
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_EP_MODULE_LOG_LEVEL);

#if defined(HW_M) || defined(HW_N) || defined(HW_N_02)
#define ZAP_DURATION_CATTLE 1000000
#define ZAP_DURATION_SHEEP 500000
#define ZAP_ON_TIME 2
#define ZAP_OFF_TIME 5
#define ZAP_FREQ (ZAP_ON_TIME + ZAP_OFF_TIME) //uS
#else
#define ZAP_DURATION_CATTLE 1000000
#define ZAP_DURATION_SHEEP 500000
#define ZAP_ON_TIME 2
#define ZAP_OFF_TIME 4
#define ZAP_FREQ 8 //uS
#endif

#define ZAP_CTRL                                                               \
	DT_GPIO_LABEL_BY_IDX(DT_NODELABEL(zap_ctrl), gpios, 0) // "GPIO_0"
#define ZAP_CTRL_PIN DT_GPIO_PIN(ZAP_CTRL_NODE, gpios)
#define ZAP_CTRL_FLAGS DT_GPIO_FLAGS(ZAP_CTRL_NODE, gpios)

const struct device *dev;

int ep_module_init(void)
{
	dev = device_get_binding(ZAP_CTRL);
	if (dev == NULL) {
		return dev;
	}

	int err = gpio_pin_configure(dev, ZAP_CTRL_PIN,
				     GPIO_OUTPUT_ACTIVE | ZAP_CTRL_FLAGS);
	if (err < 0) {
		return err;
	}
}

int ep_module_enable(bool active)
{
	int err = gpio_pin_set(dev, ZAP_CTRL_PIN, (int)active);
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
		case EP_INIT:
			ep_module_init();
			break;
		case EP_ENABLE:
			ep_module_enable(true);
			break;
		case EP_DISABLE:
			ep_module_enable(false);
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
