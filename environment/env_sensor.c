/*
 * Copyright (c) 2022 Nofence AS
 */

#include <zephyr.h>
#include "env_sensor.h"
#include "env_sensor_event.h"
#include <logging/log.h>
#include <drivers/sensor.h>

#include "error_event.h"

#include <stdio.h>

/* https://www.mouser.com/datasheet/2/783/BST-BME280-DS002-1509607.pdf */

/* C. */
#define SANITY_CHECK_TEMP_MAX 500
#define SANITY_CHECK_TEMP_MIN -200
#define SANITY_CHECK_TEMP_WARN_MAX 150
#define SANITY_CHECK_TEMP_WARN_MIN -70

/* kPa. */
#define SANITY_CHECK_PRESSURE_MAX 300
#define SANITY_CHECK_PRESSURE_MIN 10
#define SANITY_CHECK_PRESSURE_WARN_MAX 150
#define SANITY_CHECK_PRESSURE_WARN_MIN 30

/* % */
#define SANITY_CHECK_HUMIDITY_MAX 150
#define SANITY_CHECK_HUMIDITY_MIN -10
#define SANITY_CHECK_HUMIDITY_WARN_MAX 120
#define SANITY_CHECK_HUMIDITY_WARN_MIN -5

LOG_MODULE_REGISTER(env_sensor, CONFIG_ENV_SENSOR_LOG_LEVEL);
/** @brief Checks sanity of the value and submits either error or warning
 *         event to error handler based on result.
 * 
 * @param sc channel to check for.
 * @param value value to compare with min/max.
 * 
 * @returns 0 on valid sanity within WARN range, negative errno otherwise.
 */
static inline int sensor_sanity_check(enum sensor_channel sc, double value)
{
	int max = 0;
	int min = 0;

	int max_warn = 0;
	int min_warn = 0;

	switch (sc) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		max = SANITY_CHECK_TEMP_MAX;
		min = SANITY_CHECK_TEMP_MIN;

		max_warn = SANITY_CHECK_TEMP_WARN_MAX;
		min_warn = SANITY_CHECK_TEMP_WARN_MIN;
		break;
	case SENSOR_CHAN_PRESS:
		max = SANITY_CHECK_PRESSURE_MAX;
		min = SANITY_CHECK_PRESSURE_MIN;

		max_warn = SANITY_CHECK_PRESSURE_WARN_MAX;
		min_warn = SANITY_CHECK_PRESSURE_WARN_MIN;
		break;
	case SENSOR_CHAN_HUMIDITY:
		max = SANITY_CHECK_HUMIDITY_MAX;
		min = SANITY_CHECK_HUMIDITY_MIN;

		max_warn = SANITY_CHECK_HUMIDITY_WARN_MAX;
		min_warn = SANITY_CHECK_HUMIDITY_WARN_MIN;
		break;
	default:
		return -EINVAL;
	}

	if (value <= max_warn && value >= min_warn) {
		return 0;
	} else {
		/* Check if should keep it but issue a warning. */
		if (value > max || value < min) {
			char *msg = "Detected a sensor value error range";
			nf_app_error(ERR_ENV_SENSOR, -ERANGE, msg, strlen(msg));
			return -ERANGE;
		} else {
			char *msg = "Detected a sensor value warning range";
			nf_app_warning(ERR_ENV_SENSOR, -ERANGE, msg,
				       strlen(msg));
			return 0;
		}
	}
}

static inline void update_env_sensor_event_values(void)
{
	const struct device *bme_dev =
		device_get_binding(DT_LABEL(DT_NODELABEL(environment_sensor)));

	if (bme_dev == NULL) {
		char *msg = "No BME device.";
		nf_app_error(ERR_ENV_SENSOR, -ENODEV, msg, strlen(msg));
		return;
	}

	struct sensor_value temp, press, humidity;

	int err = sensor_sample_fetch(bme_dev);
	if (err) {
		char *msg = "Error fetching BME values.";
		nf_app_error(ERR_ENV_SENSOR, err, msg, strlen(msg));
		return;
	}

	err = sensor_channel_get(bme_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	if (err) {
		char *msg = "Error fetching temperature value.";
		nf_app_error(ERR_ENV_SENSOR, err, msg, strlen(msg));
		return;
	}

	err = sensor_channel_get(bme_dev, SENSOR_CHAN_PRESS, &press);
	if (err) {
		char *msg = "Error fetching pressure value.";
		nf_app_error(ERR_ENV_SENSOR, err, msg, strlen(msg));
		return;
	}

	err = sensor_channel_get(bme_dev, SENSOR_CHAN_HUMIDITY, &humidity);
	if (err) {
		char *msg = "Error fetching humidity value.";
		nf_app_error(ERR_ENV_SENSOR, err, msg, strlen(msg));
		return;
	}

	struct env_sensor_event *ev = new_env_sensor_event();

	ev->temp = sensor_value_to_double(&temp);
	err = sensor_sanity_check(SENSOR_CHAN_AMBIENT_TEMP, ev->temp);

	if (err) {
		return;
	}

	ev->press = sensor_value_to_double(&press);
	err = sensor_sanity_check(SENSOR_CHAN_PRESS, ev->press);

	if (err) {
		return;
	}

	ev->humidity = sensor_value_to_double(&humidity);
	err = sensor_sanity_check(SENSOR_CHAN_HUMIDITY, ev->humidity);

	if (err) {
		return;
	}

	EVENT_SUBMIT(ev);
}

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
	//int err;
	if (is_request_env_sensor_event(eh)) {
		update_env_sensor_event_values();
		return true;
	}
	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(env_sensor, event_handler);
EVENT_SUBSCRIBE(env_sensor, request_env_sensor_event);