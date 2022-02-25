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
#define SANITY_CHECK_TEMP_MAX 85
#define SANITY_CHECK_TEMP_MIN -40

/* kPa. */
#define SANITY_CHECK_PRESSURE_MAX 110
#define SANITY_CHECK_PRESSURE_MIN 30

/* % */
#define SANITY_CHECK_HUMIDITY_MAX 100
#define SANITY_CHECK_HUMIDITY_MIN 0

#define SENSOR_VALUE_TO_FLOAT(integer, frac)                                   \
	(float)(((float)integer) + (float)((float)frac / 1000000))

LOG_MODULE_REGISTER(env_sensor, CONFIG_ENV_SENSOR_LOG_LEVEL);

static inline int sensor_sanity_check(enum sensor_channel sc, float value)
{
	int max = 0;
	int min = 0;

	switch (sc) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		max = SANITY_CHECK_TEMP_MAX;
		min = SANITY_CHECK_TEMP_MIN;
		break;
	case SENSOR_CHAN_PRESS:
		max = SANITY_CHECK_PRESSURE_MAX;
		min = SANITY_CHECK_PRESSURE_MIN;
		break;
	case SENSOR_CHAN_HUMIDITY:
		max = SANITY_CHECK_HUMIDITY_MAX;
		min = SANITY_CHECK_HUMIDITY_MIN;
		break;
	default:
		return -EINVAL;
	}
	LOG_INF("Value is %f", value);
	if (value <= max && value >= min) {
		return 0;
	} else {
		return -ERANGE;
	}
}

static inline void update_env_sensor_event_values(void)
{
	const struct device *bme_dev =
		device_get_binding(DT_LABEL(DT_NODELABEL(environment_sensor)));

	if (bme_dev == NULL) {
		char *msg = "No BME device.";
		nf_app_error(ERR_SENDER_ENV_SENSOR, -ENODEV, msg, strlen(msg));
		return;
	}

	struct sensor_value temp, press, humidity;

	int err = sensor_sample_fetch(bme_dev);
	if (err) {
		char *msg = "Error fetching BME values.";
		nf_app_error(ERR_SENDER_ENV_SENSOR, err, msg, strlen(msg));
		return;
	}

	err = sensor_channel_get(bme_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	if (err) {
		char *msg = "Error fetching temperature value.";
		nf_app_error(ERR_SENDER_ENV_SENSOR, err, msg, strlen(msg));
		return;
	}

	err = sensor_channel_get(bme_dev, SENSOR_CHAN_PRESS, &press);
	if (err) {
		char *msg = "Error fetching pressure value.";
		nf_app_error(ERR_SENDER_ENV_SENSOR, err, msg, strlen(msg));
		return;
	}

	err = sensor_channel_get(bme_dev, SENSOR_CHAN_HUMIDITY, &humidity);
	if (err) {
		char *msg = "Error fetching humidity value.";
		nf_app_error(ERR_SENDER_ENV_SENSOR, err, msg, strlen(msg));
		return;
	}

	struct env_sensor_event *ev = new_env_sensor_event();

	ev->temp = SENSOR_VALUE_TO_FLOAT(temp.val1, temp.val2);
	err = sensor_sanity_check(SENSOR_CHAN_AMBIENT_TEMP, ev->temp);

	if (err) {
		char *msg = "Sanity check failed for temperature.";
		nf_app_error(ERR_SENDER_ENV_SENSOR, -ERANGE, msg, strlen(msg));
		return;
	}

	ev->press = SENSOR_VALUE_TO_FLOAT(press.val1, press.val2);
	err = sensor_sanity_check(SENSOR_CHAN_PRESS, ev->press);

	if (err) {
		char *msg = "Sanity check failed for pressure.";
		nf_app_error(ERR_SENDER_ENV_SENSOR, -ERANGE, msg, strlen(msg));
		return;
	}

	ev->humidity = SENSOR_VALUE_TO_FLOAT(humidity.val1, humidity.val2);
	err = sensor_sanity_check(SENSOR_CHAN_HUMIDITY, ev->humidity);

	if (err) {
		char *msg = "Sanity check failed for humidity.";
		nf_app_error(ERR_SENDER_ENV_SENSOR, -ERANGE, msg, strlen(msg));
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