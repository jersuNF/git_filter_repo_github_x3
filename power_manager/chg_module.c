#include <zephyr.h>

#include "charging.h"
#include "pwr_event.h"
#include "env_sensor_event.h"

#define MODULE charging_module
#include <logging/log.h>
LOG_MODULE_REGISTER(MODULE, CONFIG_BATTERY_LOG_LEVEL);
/* Offload charger start/stop work definitions */
static struct k_work charger_work_start;
static struct k_work charger_work_stop;

static void charging_start_work_fn(struct k_work *work);
static void charging_stop_work_fn(struct k_work *work);

static void charging_start_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	LOG_INF("Charger: Starting charging");
	charging_start();
}

static void charging_stop_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	LOG_INF("Charger: Stopping charging");
	charging_stop();
}

typedef enum charging_event_e {
	CHARGING_EVENT_TEMP_HIGH,
	CHARGING_EVENT_TEMP_OK,
	CHARGING_EVENT_BATT_HIGH,
	CHARGING_EVENT_BATT_OK
} charging_event_e;

static void charging_sm(charging_event_e chg_evt)
{
	typedef enum charging_state_e {
		CHARGING_SM_IDLE,
		CHARGING_SM_CHARGING,
	} charging_state_e;

	typedef enum charging_status_e {
		CHG_STATUS_OK = 0,
		CHG_STATUS_TEMP_HIGH = BIT(0),
		CHG_STATUS_BATT_HIGH = BIT(1)
	} charging_status_e;

	/* state is simple IDLE/CHARGING charging */
	static charging_state_e state = CHARGING_SM_IDLE;
	/* chg_status is a bitfield of chg_status_e */
	static charging_status_e chg_status = CHG_STATUS_TEMP_HIGH | CHG_STATUS_BATT_HIGH;

	/* Update chg_status depending on input */
	switch (chg_evt) {
	case CHARGING_EVENT_TEMP_HIGH:
		chg_status |= CHG_STATUS_TEMP_HIGH;
		break;
	case CHARGING_EVENT_TEMP_OK:
		chg_status &= ~CHG_STATUS_TEMP_HIGH;
		break;
	case CHARGING_EVENT_BATT_HIGH:
		chg_status |= CHG_STATUS_BATT_HIGH;
		break;
	case CHARGING_EVENT_BATT_OK:
		chg_status &= ~CHG_STATUS_BATT_HIGH;
		break;
	}

	switch (state) {
	case CHARGING_SM_IDLE:
		if (chg_status == CHG_STATUS_OK) {
			/* No high temp or high batt, start charging */
			if (!charging_in_progress()) {
				k_work_submit(&charger_work_start);
			}
			state = CHARGING_SM_CHARGING;
		}
		break;
	case CHARGING_SM_CHARGING:
		if (chg_status != CHG_STATUS_OK) {
			/* High temp or high batt activated, stop charging */
			state = CHARGING_SM_IDLE;
			if (charging_in_progress()) {
				k_work_submit(&charger_work_stop);
			}
		}
		break;
	}
}

static bool event_handler(const struct event_header *eh)
{
	if (is_pwr_status_event(eh)) {
		struct pwr_status_event *ev = cast_pwr_status_event(eh);
		if (ev->battery_mv == 0) {
			// Throw away
			return false;
		}
		if (ev->battery_mv >= CONFIG_CHARGING_BATT_THRESHOLD_STOP) {
			charging_sm(CHARGING_EVENT_BATT_HIGH);
		} else if (ev->battery_mv < CONFIG_CHARGING_BATT_THRESHOLD_START) {
			charging_sm(CHARGING_EVENT_BATT_OK);
		}
	} else if (is_env_sensor_event(eh)) {
		struct env_sensor_event *ev = cast_env_sensor_event(eh);
		if (ev->temp >= (double)CONFIG_CHARGING_TEMP_THRESHOLD_HIGH) {
			charging_sm(CHARGING_EVENT_TEMP_HIGH);
		} else if (ev->temp < (double)CONFIG_CHARGING_TEMP_THRESHOLD_OK) {
			charging_sm(CHARGING_EVENT_TEMP_OK);
		}
	}
	return false;
}

void chg_module_init()
{
	k_work_init(&charger_work_start, charging_start_work_fn);
	k_work_init(&charger_work_stop, charging_stop_work_fn);
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, env_sensor_event);
EVENT_SUBSCRIBE(MODULE, pwr_status_event);