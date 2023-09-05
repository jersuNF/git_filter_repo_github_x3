#include "nclogs.h"
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
#include "watchdog_event.h"
#include "messaging_module_events.h"
#include "stg_config.h"

#if CONFIG_CLOCK_CONTROL_NRF
#include <drivers/clock_control.h>
#include <drivers/clock_control/nrf_clock_control.h>

#define CLOCK_NODE DT_INST(0, nordic_nrf_clock)
static const struct device *clock0;
#endif

#if CONFIG_ADC_NRFX_SAADC
#include "charging.h"
#endif
#define MODULE pwr_module
#include <logging/log.h>

/* Default arguments for AT+URAT, e.g. 9,7. Will be set upon a manual BLE reset. */
#define BLE_RESET_URAT_SETTING "9,7"

LOG_MODULE_REGISTER(MODULE, CONFIG_BATTERY_LOG_LEVEL);
/* Required by generate_nclogs.py*/
#define NCID POWER_MANAGER

static uint32_t extclk_request_flags = 0;

/* Current reboot reason as read at boot, REBOOT_REASON_CNT is last entry in 
 * reboot reason list and not a valid reboot reason. */
static uint8_t m_my_reboot_reason = REBOOT_REASON_CNT;

/**
 * @brief Enable/disable external clock for HFCLK.
 * 
 * @param[in] enable Set to true to enable external clock. 
 * 
 * @return 0 on success. Otherwise a negative error code.
 */
static int pwr_module_extclk_enable(bool enable);

/** Variable to keep track of current power state */
static int current_state = PWR_LOW;

/* Define the workers for battery and charger */
static struct k_work_delayable battery_poll_work;
static struct k_work_delayable power_reboot;
#if CONFIG_ADC_NRFX_SAADC
static struct k_work_delayable charging_poll_work;
#endif

/** @brief Periodic battery voltage work function */
static void battery_poll_work_fn()
{
#if defined(CONFIG_WATCHDOG_ENABLE)
	/* Report alive */
	watchdog_report_module_alive(WDG_PWR_MODULE);
#endif
	/* Periodic log and update ble adv array */
	int batt_voltage = log_and_fetch_battery_voltage();
	if (batt_voltage < 0) {
		char *e_msg = "Error in fetching battery voltage";
		nf_app_error(ERR_PWR_MODULE, batt_voltage, e_msg, strlen(e_msg));
		return;
	}
	/* Keep old state as reference for later */
	int old_state = current_state;

	switch (old_state) {
	case PWR_NORMAL:
		if (batt_voltage < CONFIG_BATTERY_LOW - CONFIG_BATTERY_THRESHOLD) {
			current_state = PWR_LOW;
		}
		break;

	case PWR_LOW:
		if (batt_voltage < CONFIG_BATTERY_CRITICAL - CONFIG_BATTERY_THRESHOLD) {
			current_state = PWR_CRITICAL;
		} else if (batt_voltage > CONFIG_BATTERY_LOW + CONFIG_BATTERY_THRESHOLD) {
			current_state = PWR_NORMAL;
		}
		break;
	case PWR_CRITICAL:
		if (batt_voltage > (CONFIG_BATTERY_CRITICAL + CONFIG_BATTERY_THRESHOLD)) {
			current_state = PWR_LOW;
		}
		break;

	default:
		break;
	}

	/* Publish battery event with averaged voltage */
	struct pwr_status_event *event = new_pwr_status_event();
	event->pwr_state = current_state;
	event->battery_mv = batt_voltage;
	EVENT_SUBMIT(event);

	k_work_reschedule(&battery_poll_work, K_MSEC(CONFIG_BATTERY_POLLER_WORK_MSEC));
}
#if CONFIG_ADC_NRFX_SAADC
/** @brief Periodic solar charging work function */
static void charging_poll_work_fn()
{
	int charging_current_avg = charging_current_sample_averaged();
	if (charging_current_avg < 0) {
		NCLOG_ERR(NCID, TRice( iD( 3010),"err: Failed to fetch charging data %d \n", charging_current_avg));
		char *msg = "Unable fetch charging data";
		nf_app_error(ERR_PWR_MODULE, charging_current_avg, msg, strlen(msg));
		return;
	}
	struct pwr_status_event *event = new_pwr_status_event();
	event->pwr_state = PWR_CHARGING;
	event->charging_ma = charging_current_avg;
	event->battery_mv = 0;
	EVENT_SUBMIT(event);
	k_work_reschedule(&charging_poll_work, K_MSEC(CONFIG_CHARGING_POLLER_WORK_MSEC));
}
#endif

static void reboot_work_fn()
{
#ifdef CONFIG_SOC_NRF52840_QIAA
	/* Add a check that we are using NRF board
	* since they are the ones supported by nordic's <power/reboot.h>
	*/
	/* Add logic to shutdown modules if necessary. */
	sys_reboot(SYS_REBOOT_COLD);
#endif
}

int pwr_module_init(void)
{
	int err;
	/* Configure battery voltage and charging adc */
	err = battery_setup();
	if (err) {
		char *e_msg = "Failed to set up battery";
		nf_app_error(ERR_PWR_MODULE, err, e_msg, strlen(e_msg));
		return err;
	}

#if CONFIG_ADC_NRFX_SAADC
	/* Initialize and start charging */
	err = charging_setup();
	err = charging_init_module();
	if (err) {
		NCLOG_ERR(NCID, TRice( iD( 2543),"err: Failed to init charging module %d \n", err));
		char *e_msg = "Failed to configure or setup charging";
		nf_app_error(ERR_PWR_MODULE, err, e_msg, strlen(e_msg));
		return err;
	}
#endif
	current_state = PWR_LOW;

	/* NB: Battery is already initialized with SYS_INIT in battery.c */
	err = log_and_fetch_battery_voltage();
	if (err < 0) {
		char *e_msg = "Error in fetching battery voltage";
		nf_app_error(ERR_PWR_MODULE, err, e_msg, strlen(e_msg));
		return err;
	}

	/* Initialize periodic battery poll function */
	k_work_init_delayable(&battery_poll_work, battery_poll_work_fn);
	k_work_reschedule(&battery_poll_work, K_NO_WAIT);

#if CONFIG_ADC_NRFX_SAADC
	/* Initialize and start periodic charging poll function */
	k_work_init_delayable(&charging_poll_work, charging_poll_work_fn);
	k_work_reschedule(&charging_poll_work, K_NO_WAIT);
#endif

	/* Initialize the reboot function */
	k_work_init_delayable(&power_reboot, reboot_work_fn);

	/* External clock default is off */
	pwr_module_extclk_enable(false);
	extclk_request_flags = 0;

	return 0;
}

int log_and_fetch_battery_voltage(void)
{
	int batt_mV = battery_sample_averaged();
	if (batt_mV < 0) {
		NCLOG_ERR(NCID, TRice( iD( 5886),"err: Failed to read battery voltage: %d \n", batt_mV));
		return -ENOENT;
	}

	uint8_t batt_soc = battery_level_soc(batt_mV);

	struct ble_ctrl_event *event = new_ble_ctrl_event();
	event->cmd = BLE_CTRL_BATTERY_UPDATE;
	event->param.battery = batt_soc;
	EVENT_SUBMIT(event);

	NCLOG_DBG(NCID, TRice( iD( 5279),"dbg: Voltage: %d mV; State Of Charge: %u precent \n", batt_mV, batt_soc));

	return batt_mV;
}

static int pwr_module_extclk_enable(bool enable)
{
	int ret = 0;

#if CONFIG_CLOCK_CONTROL_NRF
	const char *clock_label = DT_LABEL(CLOCK_NODE);
	clock0 = device_get_binding(clock_label);
	if (enable) {
		ret = clock_control_on(clock0, CLOCK_CONTROL_NRF_SUBSYS_HF);
	} else {
		ret = clock_control_off(clock0, CLOCK_CONTROL_NRF_SUBSYS_HF);
	}
#endif

	return ret;
}

int pwr_module_use_extclk(enum pwr_requester_module req, bool use_extclk)
{
	if (use_extclk) {
		extclk_request_flags |= (1 << req);
	} else {
		extclk_request_flags &= ~(1 << req);
	}

	return pwr_module_extclk_enable(extclk_request_flags != 0);
}

int fetch_battery_percent(void)
{
	int batt_mV = battery_sample_averaged();
	if (batt_mV < 0) {
		NCLOG_ERR(NCID, TRice( iD( 4014),"err: Failed to read battery voltage: %d \n", batt_mV));
		return -ENOENT;
	}
	return battery_level_soc(batt_mV);
}

int pwr_module_reboot_reason(uint8_t *aReason)
{
	int err = 0;
	if (m_my_reboot_reason == REBOOT_REASON_CNT) {
		err = stg_config_u8_read(STG_U8_RESET_REASON, &m_my_reboot_reason);
		if (err != 0) {
			NCLOG_ERR(NCID, TRice0( iD( 4590),"err: Unable to read reboot reason from stg flash \n"));
			m_my_reboot_reason = REBOOT_UNKNOWN;
		}

		/* Set reboot reason in eeprom to REBOOT_UNKNOWN after read in 
		 * case of enexpected reboot */
		if (stg_config_u8_write(STG_U8_RESET_REASON, REBOOT_UNKNOWN) != 0) {
			NCLOG_ERR(NCID, TRice0( iD( 4259),"err: Unable to reset reboot reason in stg flash! \n"));
		}
	}
	*aReason = m_my_reboot_reason;
	return err;
}

#ifdef CONFIG_ZTEST
void __attribute__((weak)) schedule_system_reboot(void)
{
	k_work_reschedule(&power_reboot, K_SECONDS(CONFIG_SHUTDOWN_TIMER_SEC));
}
#else
void schedule_system_reboot(void)
{
	k_work_reschedule(&power_reboot, K_SECONDS(CONFIG_SHUTDOWN_TIMER_SEC));
}
#endif

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
	/* Received reboot event */
	if (is_pwr_reboot_event(eh)) {
		struct pwr_reboot_event *evt = cast_pwr_reboot_event(eh);

		int err;

		/* 0V-fix: With the introduction of FW2005 all automatic fallback to 4G RAT was removed.
		* In case we end up in a situation where we are unable to communicate on 2G, it is possible
		* to reset the collar over BLE to enable 4G again.
		*/
		if (evt->reason == REBOOT_BLE_RESET) {
			char buf[STG_CONFIG_URAT_ARG_BUF_SIZE];
			memset(buf, 0, sizeof(buf));
			strncpy(buf, BLE_RESET_URAT_SETTING, sizeof(buf) - 1);

			err = stg_config_str_write(STG_STR_MODEM_URAT_ARG, buf, sizeof(buf) - 1);
			if (err != 0) {
				LOG_WRN("Cannot reset URAT argument after BLE reset: %d", err);
			}
		}

		if ((evt->reason >= 0) && (evt->reason < REBOOT_REASON_CNT)) {
			err = stg_config_u8_write(STG_U8_RESET_REASON, (uint8_t)evt->reason);
		} else {
			err = stg_config_u8_write(STG_U8_RESET_REASON, (uint8_t)REBOOT_UNKNOWN);
		}
		if (err != 0) {
			NCLOG_ERR(NCID, TRice( iD( 4499),"err: Failed to write reboot reason to ext flash, err:%d \n", err));
		}
		NCLOG_INF(NCID, TRice( iD( 3557),"inf: Reboot event received, reason:%d \n", evt->reason));

		schedule_system_reboot();
		return false;
	}
	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);
	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE_FINAL(MODULE, pwr_reboot_event);
