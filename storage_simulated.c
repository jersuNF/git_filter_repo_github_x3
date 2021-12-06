/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>

#include "storage_event.h"
#include "ack_event.h"
#include "config_event.h"
#define MODULE storage_sim

#include <logging/log.h>
LOG_MODULE_REGISTER(MODULE, CONFIG_STORAGE_MODULE_LOG_LEVEL);

#define SENSOR_SIMULATED_THREAD_STACK_SIZE 800
#define SENSOR_SIMULATED_THREAD_PRIORITY 1
#define SENSOR_SIMULATED_THREAD_SLEEP 500


bool initialized_module = false; 

struct storage_msg_data {
	union {
		struct storage_event storage;
	} module;
};

/* Storage module super states. */
static enum state_type {
	STATE_INIT,
	STATE_IDLE,
    STATE_WRITE,
    STATE_READ,
	STATE_ERROR
} state;

static struct module_data self = {
	.name = "storage",
	.msg_q = NULL,
	.supports_shutdown = true,
};

/* Storage device. Used to identify the External Flash driver in the sensor API. */
static const struct device *flash_dev;

/* Forward declarations. */
static void message_handler(struct storage_msg_data *data);

/* Convenience functions used in internal state handling. */
static char *state2str(enum state_type new_state)
{
	switch (new_state) {
	case STATE_INIT:
		return "STATE_INIT";
	case STATE_IDLE:
		return "STATE_IDLE";
	case STATE_WRITE:
		return "STATE_WRITE";
    case STATE_READ:
		return "STATE_READ";
    case STATE_ERROR:
		return "STATE_ERROR";
	default:
		return "Unknown";
	}
}

static void state_set(enum state_type new_state)
{
	if (new_state == state) {
		LOG_DBG("State: %s", state2str(state));
		return;
	}

	LOG_DBG("State transition %s --> %s",
		state2str(state),
		state2str(new_state));

	state = new_state;
}

static K_THREAD_STACK_DEFINE(sensor_simulated_thread_stack,
			     SENSOR_SIMULATED_THREAD_STACK_SIZE);
static struct k_thread sensor_simulated_thread;

static int32_t serial_number;

static void read_serial_number(void)
{

	struct storage_event *storage_event = new_storage_event();

	storage_event->data.pvt.serial_number = serial_number;
	EVENT_SUBMIT(storage_event);
}

static void sensor_simulated_thread_fn(void)
{
	while (true) {
		read_serial_number();
		k_sleep(K_MSEC(SENSOR_SIMULATED_THREAD_SLEEP));
	}
}

static void init(void)
{
	k_thread_create(&sensor_simulated_thread,
			sensor_simulated_thread_stack,
			SENSOR_SIMULATED_THREAD_STACK_SIZE,
			(k_thread_entry_t)sensor_simulated_thread_fn,
			NULL, NULL, NULL,
			SENSOR_SIMULATED_THREAD_PRIORITY,
			0, K_NO_WAIT);
}

/* Set state */
static void state_set(enum state_type new_state)
{
	if (new_state == state) {
		LOG_DBG("State: %s", state2str(state));
		return;
	}

	LOG_DBG("State transition %s --> %s",
		state2str(state),
		state2str(new_state));

	state = new_state;
}

/* Message handler for all states. */
static void on_all_states(struct storage_msg_data *msg)
{
	if (IS_EVENT(msg, app, APP_EVT_START)) {
		int err;

		err = module_start(&self);
		if (err) {
			LOG_ERR("Failed starting module, error: %d", err);
			SEND_ERROR(gps, GPS_EVT_ERROR, err);
		}

		state_set(STATE_INIT);

		err = setup();
		if (err) {
			LOG_ERR("setup, error: %d", err);
			SEND_ERROR(gps, GPS_EVT_ERROR_CODE, err);
		}
	}

	if (IS_EVENT(msg, util, UTIL_EVT_SHUTDOWN_REQUEST)) {
		/* The module doesn't have anything to shut down and can
		 * report back immediately.
		 */
		SEND_SHUTDOWN_ACK(gps, GPS_EVT_SHUTDOWN_READY, self.id);
		state_set(STATE_SHUTDOWN);
	}
}

/* Handlers */
static bool event_handler(const struct event_header *eh)
{

    /* Receive a configuration event */
	if (is_storage_event(eh)) {
		struct storage_event *event = cast_storage_event(eh);
        struct storage_msg_data msg = {
            .module.storage = *event
        };
        message_handler(&msg);

    }
    /* Init thread to read serial number */
    init();

    return false;
	}

}


/* Message handler for STATE_INIT. */
static void on_state_init(struct gps_msg_data *msg)
{

	/* This boolean shuld be passed from a separate event, ref asset tracker */
	if (!initialized_module) {
		gps_cfg.timeout = msg->module.data.data.cfg.gps_timeout;
		state_set(STATE_IDLE);
	}
	initialized_module = true;
}

/* Message handler for STATE_RUNNING. */
static void on_state_running(struct gps_msg_data *msg)
{
	
}

static void message_handler(struct gps_msg_data *msg)
{
	switch (state) {
	case STATE_INIT:
		on_state_init(msg);
		break;
	case STATE_IDLE:
		on_state_running(msg);
		break;
    case STATE_READ:
		/* Read from flash */
		break;
    case STATE_WRITE:
		/* Write to flash */
		break;
	case STATE_ERROR:
		/* The error state has no transition. */
		break;
	default:
		LOG_ERR("Unknown storage module state.");
		break;
	}

	on_all_states(msg);
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, config_event);
EVENT_SUBSCRIBE(MODULE, storage_event);
