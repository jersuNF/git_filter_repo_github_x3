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
#include "ble_ctrl_event.h"

#define MODULE pwr_module
#include <logging/log.h>

LOG_MODULE_REGISTER(MODULE, CONFIG_PWR_MODULE_LOG_LEVEL);

/** Variable to keep track of current power state */
static int current_state = PWR_NORMAL;

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

/** @brief Periodic battery voltage work function */
static void battery_poll_work_fn()
{
	/* Periodic log and update ble adv array */
	int batt_voltage = log_and_fetch_battery_voltage();
	if (batt_voltage < 0) {
		char *e_msg = "Error in fetching battery voltage";
		nf_app_error(ERR_PWR_MODULE, batt_voltage, e_msg,
			     strlen(e_msg));
		return;
	}
	if (batt_voltage < CONFIG_BATTRY_LOW_THRESHOLD &&
	    batt_voltage > CONFIG_BATTRY_CRITICAL_THRESHOLD) {
		struct pwr_status_event *event = new_pwr_status_event();
		event->pwr_state = PWR_LOW;
		EVENT_SUBMIT(event);
		current_state = PWR_LOW;

	} else if (batt_voltage < CONFIG_BATTRY_CRITICAL_THRESHOLD) {
		struct pwr_status_event *event = new_pwr_status_event();
		event->pwr_state = PWR_CRITICAL;
		EVENT_SUBMIT(event);
		current_state = PWR_CRITICAL;

	} else if (batt_voltage >= (CONFIG_BATTRY_NORMAL_THRESHOLD) &&
		   current_state != PWR_NORMAL) {
		/* Avoid sending state change if PWR state is normal */
		struct pwr_status_event *event = new_pwr_status_event();
		event->pwr_state = PWR_NORMAL;
		EVENT_SUBMIT(event);
		current_state = PWR_NORMAL;
	}
	k_work_reschedule(&battery_poll_work,
			  K_SECONDS(CONFIG_BATTRY_POLLER_WORK_SEC));
}

int pwr_module_init(void)
{
	/* Set PWR state to NORMAL as initial state */
	struct pwr_status_event *event = new_pwr_status_event();
	event->pwr_state = PWR_NORMAL;
	EVENT_SUBMIT(event);
	current_state = PWR_NORMAL;

	/* NB: Battery is already initialized with SYS_INIT in battery.c */
	int err = log_and_fetch_battery_voltage();
	if (err < 0) {
		char *e_msg = "Error in fetching battery voltage";
		nf_app_error(ERR_PWR_MODULE, err, e_msg, strlen(e_msg));
		return err;
	}

	/* Initialize periodic battery poll function */
	k_work_init_delayable(&battery_poll_work, battery_poll_work_fn);
	k_work_reschedule(&battery_poll_work,
			  K_SECONDS(CONFIG_BATTRY_POLLER_WORK_SEC));

	return 0;
}

int log_and_fetch_battery_voltage(void)
{
	int batt_mV = battery_sample();
	if (batt_mV < 0) {
		LOG_ERR("Failed to read battery voltage: %d", batt_mV);
		return -ENOENT;
	}

	uint8_t batt_soc = battery_level_soc(batt_mV, levels);

	struct ble_ctrl_event *event = new_ble_ctrl_event();
	event->cmd = BLE_CTRL_BATTERY_UPDATE;
	event->param.battery = batt_soc;
	EVENT_SUBMIT(event);

	LOG_INF("Voltage: %d mV; State Of Charge: %u precent", batt_mV,
		batt_soc);

	return batt_mV;
}
