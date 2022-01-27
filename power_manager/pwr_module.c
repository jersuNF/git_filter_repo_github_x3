/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include <logging/log.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include "pwr_module.h"
#include "pwr_event.h"
#include "error_event.h"

#define MODULE pwr_module
#include <logging/log.h>

LOG_MODULE_REGISTER(MODULE, CONFIG_PWR_MODULE_LOG_LEVEL);

int pwr_module_init(void)
{
	// TODO: Implement initialization of the pwr module
	return 0;
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
	if (is_pwr_status_event(eh)) {
		const struct pwr_status_event *event =
			cast_pwr_status_event(eh);
		switch (event->pwr_state) {
		case PWR_IDLE:
			break;
		case PWR_ACTIVE:
			break;
		case PWR_SLEEP:
			break;
		case PWR_OFF:
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
EVENT_SUBSCRIBE(MODULE, pwr_status_event);