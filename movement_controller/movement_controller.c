/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include "movement_controller.h"
#include "movement_events.h"
#include <logging/log.h>
#include <drivers/sensor.h>

#define LOG_MODULE_NAME move_controller
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_MOVE_CONTROLLER_LOG_LEVEL);

static const struct device *sensor;

void movement_thread_fn();
K_THREAD_DEFINE(movement_thread, CONFIG_MOVEMENT_THREAD_STACK_SIZE,
		movement_thread_fn, NULL, NULL, NULL,
		CONFIG_MOVEMENT_THREAD_PRIORITY, 0, 0);

K_MSGQ_DEFINE(acc_data_msgq, sizeof(raw_acc_data_t), CONFIG_ACC_MSGQ_SIZE, 4);

int process_acc_data(raw_acc_data_t *acc)
{
	if (acc == NULL) {
		LOG_ERR("No data available.");
		return -ENODATA;
	}

	uint16_t stepcount = 0;
	acc_mode_t mode = ACTIVITY_NO;

	/* accel2.c logic here. */

	struct movement_out_event *event = new_movement_out_event();
	event->activity = mode;
	event->stepcount = stepcount;

	EVENT_SUBMIT(event);
	return 0;
}

void movement_thread_fn()
{
	while (true) {
		raw_acc_data_t raw_data;
		int err = k_msgq_get(&acc_data_msgq, &raw_data, K_FOREVER);
		if (err) {
			LOG_ERR("Error retrieving accelerometer message queue %i",
				err);
			continue;
		}
		err = process_acc_data(&raw_data);
		if (err) {
			LOG_ERR("Error processing accelerometer data. %i", err);
		}
	}
}

void fetch_and_display(const struct device *sensor)
{
	struct sensor_value accel[3];
	int rc = sensor_sample_fetch(sensor);

	if (rc < 0) {
		LOG_ERR("Error fetching acc sensor values %i", rc);
		return;
	}

	rc = sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_XYZ, accel);

	if (rc < 0) {
		LOG_ERR("Error getting acc channel values %i", rc);
		return;
	}

	raw_acc_data_t data;
	data.x = sensor_value_to_double(&accel[0]);
	data.y = sensor_value_to_double(&accel[1]);
	data.z = sensor_value_to_double(&accel[2]);

	LOG_DBG("Acc X: %f, Y: %f, Z: %f", data.x, data.y, data.z);

	while (k_msgq_put(&acc_data_msgq, &data, K_NO_WAIT) != 0) {
		/* Message queue is full: purge old data & try again */
		k_msgq_purge(&acc_data_msgq);
	}
}

/* Interrupt trigger function. */
static void trigger_handler(const struct device *dev,
			    const struct sensor_trigger *trig)
{
	fetch_and_display(dev);
	/** @todo Resample logic to 1hz / 10hz. */
}

int update_acc_odr_and_trigger(acc_mode_t mode_hz)
{
	/* Setup interrupt triggers. */
	struct sensor_trigger trig;
	int rc;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;

	uint32_t hz = 0;

	switch (mode_hz) {
	case MODE_OFF: {
		hz = 0;
		break;
	}
	case MODE_1_6_HZ: {
		hz = 1;
		break;
	}
	case MODE_12_5_HZ: {
		hz = 12;
		break;
	}
	default: {
		return -EINVAL;
	}
	}

	struct sensor_value odr = {
		.val1 = hz,
	};

	rc = sensor_attr_set(sensor, trig.chan, SENSOR_ATTR_SAMPLING_FREQUENCY,
			     &odr);
	if (rc != 0) {
		LOG_ERR("Failed to set odr: %d", rc);
		return rc;
	}

	rc = sensor_trigger_set(sensor, &trig, trigger_handler);
	if (rc != 0) {
		LOG_ERR("Failed to set trigger: %d", rc);
		return rc;
	}

	return 0;
}

int init_movement_controller(void)
{
	int err;

	sensor = device_get_binding(DT_LABEL(DT_NODELABEL(movement_sensor)));

	if (sensor == NULL) {
		LOG_ERR("Could not find LIS2DW driver.");
		return -ENODEV;
	}
	if (!device_is_ready(sensor)) {
		LOG_ERR("Failed to setup LIS2DW accelerometer driver.");
		return -EFAULT;
	} else {
		LOG_INF("Setup LIS2DW accelerometer driver.");
	}

	/* Setup interrupt triggers. */
	err = update_acc_odr_and_trigger(MODE_1_6_HZ);
	if (err) {
		return err;
	}
	return 0;
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
	int err;
	if (is_movement_set_mode_event(eh)) {
		struct movement_set_mode_event *ev =
			cast_movement_set_mode_event(eh);
		err = update_acc_odr_and_trigger(ev->acc_mode);
		if (err) {
			return false;
		}
		return false;
	}
	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, movement_set_mode_event);