/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include <logging/log.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "pwr_module.h"
#include "pwr_event.h"
#include "error_event.h"
#include "battery.h"

#define MODULE pwr_module
#include <logging/log.h>

LOG_MODULE_REGISTER(MODULE, CONFIG_PWR_MODULE_LOG_LEVEL);

/** A discharge curve specific to the power source. */
static const struct battery_level_point levels[] = {
#if DT_NODE_HAS_PROP(DT_INST(0, voltage_divider), io_channels)
	/* "Curve" here eyeballed from captured data for the [Adafruit
	 * 3.7v 2000 mAh](https://www.adafruit.com/product/2011) LIPO
	 * under full load that started with a charge of 3.96 V and
	 * dropped about linearly to 3.58 V over 15 hours.  It then
	 * dropped rapidly to 3.10 V over one hour, at which point it
	 * stopped transmitting.
	 *
	 * Based on eyeball comparisons we'll say that 15/16 of life
	 * goes between 3.95 and 3.55 V, and 1/16 goes between 3.55 V
	 * and 3.1 V.
	 */

	{ 10000, 3950 },
	{ 625, 3550 },
	{ 0, 3100 },
#else
	/* Linear from maximum voltage to minimum voltage. */
	{ 10000, 3600 },
	{ 0, 1700 },
#endif
};

static const char *now_str(void)
{
	static char buf[16]; /* ...HH:MM:SS.MMM */
	uint32_t now = k_uptime_get_32();
	unsigned int ms = now % MSEC_PER_SEC;
	unsigned int s;
	unsigned int min;
	unsigned int h;

	now /= MSEC_PER_SEC;
	s = now % 60U;
	now /= 60U;
	min = now % 60U;
	now /= 60U;
	h = now;

	snprintf(buf, sizeof(buf), "%u:%02u:%02u.%03u", h, min, s, ms);
	return buf;
}

int pwr_module_init(void)
{
	// TODO: Implement initialization of the pwr module
	return 0;
}

void fetch_periodic_battery_voltage(void)
{
	int rc = battery_measure_enable(true);

	if (rc != 0) {
		LOG_ERR("Failed initialize battery measurement: %d", rc);
		return;
	}

	while (true) {
		int batt_mV = battery_sample();

		if (batt_mV < 0) {
			LOG_ERR("Failed to read battery voltage: %d", batt_mV);
			break;
		}

		unsigned int batt_pptt = battery_level_pptt(batt_mV, levels);

		LOG_INF("[%s]: %d mV; %u pptt", log_strdup(now_str()), batt_mV,
			batt_pptt);

		k_busy_wait(5 * USEC_PER_SEC);
	}
	rc = battery_measure_enable(false);
	LOG_INF("Disable measurement: %d", rc);
}

/*
TODO: NOTES from meeting 28.1:
Read battery func()
	IF battery level is low: {
	Publish power state event -> LOW_PWR_MODE
		inside cellular_handler:
			Safely turn down modem, send goodbye to server

		inside flash / GNSS module:
			Turn down flash? Shared witch with GNSS

	Publish error event -> WARNING LOW BATTERY
}
*/

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
