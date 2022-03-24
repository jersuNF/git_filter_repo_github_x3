/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/watchdog.h>

#include "watchdog_app.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(watchdog, CONFIG_WATCHDOG_LOG_LEVEL);

#define WDT_FEED_WORKER_DELAY_MS ((CONFIG_WATCHDOG_TIMEOUT_SEC * 1000) / 2)
#define WATCHDOG_TIMEOUT_MSEC (CONFIG_WATCHDOG_TIMEOUT_SEC * 1000)

struct wdt_data_storage {
	const struct device *wdt_drv;
	int wdt_channel_id;
	struct k_work_delayable system_workqueue_work;
};

/* Flag set when the library has been initialized and started. */
static bool init_and_start;

static struct wdt_data_storage wdt_data;

static void primary_feed_worker()
{
	int err = wdt_feed(wdt_data.wdt_drv, wdt_data.wdt_channel_id);

	if (err) {
		LOG_ERR("Cannot feed watchdog. Error code: %d", err);
		return;
	}

	LOG_INF("Feeding watchdog.");
}

static int watchdog_timeout_install(void)
{
	static const struct wdt_timeout_cfg wdt_settings = {
		.window = {
			.min = 0,
			.max = WATCHDOG_TIMEOUT_MSEC,
		},
		.callback = NULL,
		.flags = WDT_FLAG_RESET_SOC
	};

	wdt_data.wdt_channel_id =
		wdt_install_timeout(wdt_data.wdt_drv, &wdt_settings);
	if (wdt_data.wdt_channel_id < 0) {
		LOG_ERR("Cannot install watchdog timer! Error code: %d",
			wdt_data.wdt_channel_id);
		return -EFAULT;
	}

	LOG_INF("Watchdog timeout set. Timeout: %d seconds",
		CONFIG_WATCHDOG_TIMEOUT_SEC);
	return 0;
}

static int watchdog_start(void)
{
	int err = wdt_setup(wdt_data.wdt_drv, WDT_OPT_PAUSE_HALTED_BY_DBG);

	if (err) {
		LOG_ERR("Cannot start watchdog! Error code: %d", err);
	} else {
		LOG_INF("Watchdog started");
	}
	return err;
}

void external_watchdog_feed(void)
{
	if (init_and_start) {
		k_work_schedule(&wdt_data.system_workqueue_work,
				K_MSEC(WDT_FEED_WORKER_DELAY_MS));
	}
}

static int watchdog_feed_enable(void)
{
	k_work_init_delayable(&wdt_data.system_workqueue_work,
			      primary_feed_worker);

	int err = wdt_feed(wdt_data.wdt_drv, wdt_data.wdt_channel_id);

	if (err) {
		LOG_ERR("Cannot feed watchdog. Error code: %d", err);
		return err;
	}

	return err;
}

static int watchdog_enable(void)
{
	int err = -ENXIO;

	wdt_data.wdt_drv = device_get_binding(DT_LABEL(DT_NODELABEL(wdt)));
	if (wdt_data.wdt_drv == NULL) {
		LOG_ERR("Cannot bind watchdog driver, %d", err);
		return err;
	}

	err = watchdog_timeout_install();
	if (err) {
		return err;
	}

	err = watchdog_start();
	if (err) {
		return err;
	}

	err = watchdog_feed_enable();
	if (err) {
		return err;
	}

	return 0;
}

int watchdog_init_and_start(void)
{
	int err;

	err = watchdog_enable();
	if (err) {
		LOG_ERR("Failed to enable watchdog, error: %d", err);
		return err;
	}

	init_and_start = true;
	return 0;
}