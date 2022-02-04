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
	/* "Curve" here eyeballed from captured data for example cell [Adafruit
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

	{ 10000, 4200 },
	{ 625, 3550 },
	{ 0, 3100 },

};

static struct k_work_delayable battery_poll_work;

void battery_poll_work_fn()
{
	/* Add logic for periodic update of ble adv array in ble module */
	int err = log_battery_voltage();
	if (err < 0) {
		char *e_msg = "Error in fetching battery voltage";
		nf_app_error(ERR_PWR_MODULE, err, e_msg, strlen(e_msg));
	}
	k_work_reschedule(&battery_poll_work,
			  K_SECONDS(CONFIG_BATTRY_POLLER_WORK_SEC));
}

int pwr_module_init(void)
{
	/* NB: Battery is already initialized with SYS_INIT in battery.c */
	int err = log_battery_voltage();
	if (err < 0) {
		char *e_msg = "Error in fetching battery voltage";
		nf_app_error(ERR_PWR_MODULE, err, e_msg, strlen(e_msg));
	}
	k_work_init_delayable(&battery_poll_work, battery_poll_work_fn);
	k_work_reschedule(&battery_poll_work,
			  K_SECONDS(CONFIG_BATTRY_POLLER_WORK_SEC));

	/* Set PWR state to NORMAL */
	struct pwr_status_event *event = new_pwr_status_event();
	event->pwr_state = PWR_NORMAL;
	EVENT_SUBMIT(event);
	return err;
}

int log_battery_voltage(void)
{
	int err = battery_measure_enable(true);

	if (err != 0) {
		LOG_ERR("Failed initialize battery measurement: %d", err);
		return err;
	}

	int batt_mV = battery_sample();
	if (batt_mV < 0) {
		LOG_ERR("Failed to read battery voltage: %d", batt_mV);
		return err;
	}

	unsigned int batt_soc = battery_level_soc(batt_mV, levels);

	LOG_INF("Voltage: %d mV; State Of Charge: %u precent", batt_mV,
		batt_soc);

	/* Disable gipo pin to save power */
	err = battery_measure_enable(false);
	if (err != 0) {
		LOG_ERR("Failed to disable battery measurement: %d", err);
		return err;
	}
	return 0;
}

void fetch_periodic_battery_voltage(void)
{
	int err = battery_measure_enable(true);

	if (err != 0) {
		LOG_ERR("Failed initialize battery measurement: %d", err);
		return;
	}

	while (true) {
		int batt_mV = battery_sample();

		if (batt_mV < 0) {
			LOG_ERR("Failed to read battery voltage: %d", batt_mV);
			break;
		}

		unsigned int batt_soc = battery_level_soc(batt_mV, levels);

		LOG_INF("Voltage: %d mV; State Of Charge: %u precent", batt_mV,
			batt_soc);

		k_busy_wait(5 * USEC_PER_SEC);
	}

	/* Disable gipo pin to save power */
	err = battery_measure_enable(false);
	if (err != 0) {
		LOG_ERR("Failed to disable battery measurement: %d", err);
		return;
	}
	LOG_INF("Disable measurement: %d", err);
}