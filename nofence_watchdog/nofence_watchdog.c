/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include <device.h>
#include <string.h>
#include <errno.h>
#include <logging/log.h>
#define MODULE nofence_wdt
LOG_MODULE_REGISTER(MODULE, CONFIG_LOG_DEFAULT_LEVEL);
typedef struct nofence_wdt_module_t {
	char name[CONFIG_SIZE_WDT_MODULE_NAMES];
	bool kicked;
	bool active;
	uint8_t reason;
} nofence_wdt_module_t;

typedef struct nofence_wdt_modules_t {
	nofence_wdt_module_t modules[CONFIG_NUM_WDT_MODULES];
	void (*timer_cb)(uint8_t);
	uint32_t timer_time_seconds;
} nofence_wdt_modules_t;

static nofence_wdt_modules_t wdt_modules;

static void wdt_timer_cb(struct k_timer *timer);

K_TIMER_DEFINE(wdt_timer, wdt_timer_cb, NULL);

static void wdt_timer_cb(struct k_timer *timer)
{
	for (size_t i = 0; i < CONFIG_NUM_WDT_MODULES; i++) {
		if (wdt_modules.modules[i].active && !wdt_modules.modules[i].kicked) {
			/* One of the registered timer didn't kick */
			if (wdt_modules.timer_cb != NULL) {
				wdt_modules.timer_cb(wdt_modules.modules[i].reason);
			}
		}
		/* reset kicked for next pass */
		wdt_modules.modules[i].kicked = false;
	}
	k_timer_start(&wdt_timer, K_SECONDS(wdt_modules.timer_time_seconds), K_NO_WAIT);
}

void nofence_wdt_register_cb(void (*cb)(uint8_t rst_reason))
{
	wdt_modules.timer_cb = cb;
}

void nofence_wdt_init()
{
	/* Stop timer in case this init is done several times */
	k_timer_stop(&wdt_timer);
	memset(&wdt_modules, 0, sizeof(wdt_modules));
}

static void wdt_module_setup(nofence_wdt_module_t *module, uint8_t reason, uint32_t wdt_timer_s)
{
	module->reason = reason;
	module->active = true;
	module->kicked = false;
	if (wdt_timer_s > wdt_modules.timer_time_seconds) {
		/* This wdt timer runs at the slowest registered wdt timer interval */
		wdt_modules.timer_time_seconds = wdt_timer_s;
		/* extend timer to the registered module time */
		k_timer_start(&wdt_timer, K_SECONDS(wdt_modules.timer_time_seconds), K_NO_WAIT);
	}
}

int nofence_wdt_module_register(const char *module, uint8_t reason, uint32_t wdt_timer_s)
{
	if (strlen(module) >= CONFIG_SIZE_WDT_MODULE_NAMES) {
		return -EINVAL;
	}

	/* First, check if module already exist*/
	for (size_t i = 0; i < CONFIG_NUM_WDT_MODULES; i++) {
		if (strcmp(module, wdt_modules.modules[i].name) == 0) {
			wdt_module_setup(&wdt_modules.modules[i], reason, wdt_timer_s);
			return 0;
		}
	}

	/* Module doesn't already exist, setup a new one */
	for (size_t i = 0; i < CONFIG_NUM_WDT_MODULES; i++) {
		if (wdt_modules.modules[i].name[0] == '\0') {
			memcpy(wdt_modules.modules[i].name, module, strlen(module));
			wdt_module_setup(&wdt_modules.modules[i], reason, wdt_timer_s);
			return 0;
		}
	}
	return -EINVAL;
}

int nofence_wdt_module_unregister(const char *module)
{
	for (size_t i = 0; i < CONFIG_NUM_WDT_MODULES; i++) {
		if (strcmp(module, wdt_modules.modules[i].name) == 0) {
			memset(wdt_modules.modules[i].name, '\0', CONFIG_SIZE_WDT_MODULE_NAMES);
			wdt_modules.modules[i].kicked = false;
			wdt_modules.modules[i].active = false;
			return 0;
		}
	}
	return -EINVAL;
}

int nofence_wdt_kick(char *module)
{
	for (size_t i = 0; i < CONFIG_NUM_WDT_MODULES; i++) {
		if (strcmp(module, wdt_modules.modules[i].name) == 0) {
			wdt_modules.modules[i].kicked = true;
			return 0;
		}
	}
	return -EINVAL;
}