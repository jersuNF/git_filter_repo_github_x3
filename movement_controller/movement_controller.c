/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include "movement_controller.h"
#include "movement_events.h"
#include <logging/log.h>
#include <drivers/sensor.h>

#include "nf_fifo.h"
#include "trigonometry.h"

LOG_MODULE_REGISTER(move_controller, CONFIG_MOVE_CONTROLLER_LOG_LEVEL);

static const struct device *sensor;

#define ACC_FIFO_ELEMENTS 32

/** @todo Add to make configurable from messaging.c event. Also eeprom settings? */
static uint32_t off_animal_time_limit_sec = OFF_ANIMAL_TIME_LIMIT_SEC_DEFAULT;
static uint32_t acc_sigma_sleep_limit = ACC_SIGMA_SLEEP_LIMIT_DEFAULT;
static uint32_t acc_sigma_noactivity_limit = ACC_SIGMA_NOACTIVITY_LIMIT_DEFAULT;

static int16_t acc_fifo_x[ACC_FIFO_ELEMENTS];
static int16_t acc_fifo_y[ACC_FIFO_ELEMENTS];
static int16_t acc_fifo_z[ACC_FIFO_ELEMENTS];

static uint32_t first_inactive_timestamp = 0;
static uint32_t total_steps = 0;
static uint16_t activity_decrease_timestamp = 0;

static movement_state_t prev_state = STATE_INACTIVE;

static uint8_t num_acc_fifo_samples = 0;

typedef enum {
	ACTIVITY_NO = 0,
	ACTIVITY_LOW = 1,
	ACTIVITY_MED = 2,
	ACTIVITY_HIGH = 3
} acc_activity_t;
static acc_activity_t last_activity = ACTIVITY_NO;

/* Variable used to check if we time out regarding new accelerometer data. */

void movement_timeout_fn(struct k_timer *dummy);
K_TIMER_DEFINE(movement_timeout_timer, movement_timeout_fn, NULL);

void movement_thread_fn();
K_THREAD_DEFINE(movement_thread, CONFIG_MOVEMENT_THREAD_STACK_SIZE,
		movement_thread_fn, NULL, NULL, NULL,
		CONFIG_MOVEMENT_THREAD_PRIORITY, 0, 0);

K_MSGQ_DEFINE(acc_data_msgq, sizeof(raw_acc_data_t), CONFIG_ACC_MSGQ_SIZE, 4);

/** @brief Times out after n seconds of not receiving new data. */
void movement_timeout_fn(struct k_timer *dummy)
{
	ARG_UNUSED(dummy);

	struct movement_timeout_event *ev = new_movement_timeout_event();
	EVENT_SUBMIT(ev);
}

/** @brief Counts the number of steps in the FIFO-queue. Works only with 10Hz.
 * 
 * @param gravity z axis values normalized.
 * 
 * @returns The number of steps in the FIFO-queue
 */
uint8_t acc_count_steps(int32_t gravity)
{
	uint8_t i;
	int16_t acceleration[ACC_FIFO_ELEMENTS];
	uint8_t step_counter = 0;
	bool step_flag = false;

	/* Compute gravity and animal component. */
	for (i = 0; i < ACC_FIFO_ELEMENTS; i++) {
		acceleration[i] = -(acc_fifo_z[i] - gravity);
	}

	/* Count the number of steps in the FIFO-queue. */
	for (i = 0; i < ACC_FIFO_ELEMENTS; i++) {
		if (acceleration[i] < 0 && step_flag == false) {
			step_flag = true;
		}
		if (acceleration[i] > STEP_THRESHOLD && step_flag == true) {
			step_flag = false;
			step_counter++;
		}
	}
	return step_counter;
}

void reset_total_steps(void)
{
	total_steps = 0;
}

void process_acc_data(raw_acc_data_t *acc)
{
	if (acc == NULL) {
		/* Error handle. */
		LOG_ERR("No data available.");
		return;
	}

	/* Gets set to true when we fill up the fifo. */
	static bool first_read = true;
	/* Store newest value into FIFO. */
	fifo_put(acc->x, acc_fifo_x, ACC_FIFO_ELEMENTS);
	fifo_put(acc->y, acc_fifo_y, ACC_FIFO_ELEMENTS);
	fifo_put(acc->z, acc_fifo_z, ACC_FIFO_ELEMENTS);

	if (++num_acc_fifo_samples < ACC_FIFO_ELEMENTS) {
		/* Error/warning handle. */
		LOG_DBG("Cannot calculate since FIFO is not filled.");
		return;
	}

	/* FIFO is filled, start algorithm. */
	bool is_active = true;
	uint8_t cur_activity;
	uint8_t stepcount;
	uint8_t i;
	uint32_t acc_norm[ACC_FIFO_ELEMENTS];
	int32_t gravity = 0;

	/* Compute ||Norm|| of accelerometer X,Y,Z points
	 * Max is 2g, so max NORM in x,y,z is SQRT(4+4+4) = 3.4641 g = 3464 mg
	 * Due to existing scaling bug, 8 bit values are returned in int16_t so values
	 * are times 256. Since one "tick" is 16 mg, the max theoretical value is
	 * math.sqrt(4+4+4) * 1000 * 256 / 16 = 55425.62584220407 ~ 55426.
         */

	uint32_t acc_norm_sum = 0;

	for (i = 0; i < ACC_FIFO_ELEMENTS; i++) {
		uint32_t acc_tot = ((uint32_t)acc_fifo_x[i] * acc_fifo_x[i]) +
				   ((uint32_t)acc_fifo_y[i] * acc_fifo_y[i]) +
				   ((uint32_t)acc_fifo_z[i] * acc_fifo_z[i]);
		acc_norm[i] = g_u32_SquareRootRounded(acc_tot);
		acc_norm_sum += acc_norm[i];

		gravity += acc_fifo_z[i] / ACC_FIFO_ELEMENTS;
	}

	uint32_t acc_norm_mean = acc_norm_sum / ACC_FIFO_ELEMENTS;

	/* Compute Standard Deviation. */
	uint32_t acc_std = 0;
	uint32_t acc_std_final = 0;
	for (i = 0; i < ACC_FIFO_ELEMENTS; i++) {
		int32_t x = (acc_norm[i] - acc_norm_mean);
		acc_std += (x * x);
	}
	acc_std /= ACC_FIFO_ELEMENTS;
	acc_std = g_u32_SquareRootRounded(acc_std);

	/* Exponential moving average with alpha = 2/(N+1). */
	if (first_read) {
		first_read = false;
		acc_std_final = acc_std;
	} else {
		acc_std_final =
			(acc_std * 2) / (ACC_STD_EXP_MOVING_AVERAGE_N + 1) +
			acc_std_final -
			(acc_std_final * 2) /
				(ACC_STD_EXP_MOVING_AVERAGE_N + 1);
	}

	/* Determine current activity level, the number below is based 
         * on 8 HW_F, HW_J collars placed outside on a flower-bed at 
         * OH5 in windy conditions, and observing their collar status over time.
         */
	if (acc_std < acc_sigma_noactivity_limit) {
		cur_activity = ACTIVITY_NO;
		stepcount = 0;
	} else {
		stepcount = acc_count_steps(gravity);
		if (stepcount == 0) {
			cur_activity = ACTIVITY_LOW;
		} /* resting or grazing. */
		else if (stepcount < STEPS) {
			cur_activity = ACTIVITY_MED;
		} /* walking. */
		else {
			cur_activity = ACTIVITY_HIGH;
		} /* running. */
	}

	/** @todo Use total steps? */
	total_steps += stepcount;

	/* Gradually increase or decrease of activity level. 
         * If activity is greater than the last, increment instantly.
         */
	uint32_t a_delta = k_uptime_get_32() - activity_decrease_timestamp;
	if (cur_activity > last_activity) {
		cur_activity = last_activity + 1;
		activity_decrease_timestamp = k_uptime_get_32();
	} else if (cur_activity < last_activity) {
		if ((last_activity == ACTIVITY_LOW &&
		     a_delta > A_DEC_THRESHOLD_NO) ||
		    (last_activity == ACTIVITY_MED &&
		     a_delta > A_DEC_THRESHOLD_LOW) ||
		    (last_activity == ACTIVITY_HIGH &&
		     a_delta > A_DEC_THRESHOLD_MED)) {
			cur_activity = last_activity - 1;
			activity_decrease_timestamp = k_uptime_get_32();
		} else {
			cur_activity = last_activity;
		}
	} else {
		activity_decrease_timestamp = k_uptime_get_32();
	}

	/* Set timestamp for when we entered no activity. */
	if (cur_activity == ACTIVITY_NO && last_activity > ACTIVITY_NO) {
		first_inactive_timestamp = k_uptime_get_32();
	}

	if (cur_activity == ACTIVITY_NO) {
		uint32_t inactive_for_sec =
			k_uptime_get_32() - first_inactive_timestamp;
		if (inactive_for_sec >= off_animal_time_limit_sec) {
			is_active = false;
		}
	}

	/* Update current activity level. */
	last_activity = cur_activity;

	/* Determine the movement state. */
	movement_state_t m_state = STATE_SLEEP;
	if (!is_active) {
		m_state = STATE_INACTIVE;
	} else {
		if (acc_std_final >= acc_sigma_sleep_limit) {
			m_state = STATE_NORMAL;
		}
	}

	/* Submit activity mode and stepcount event if have a new state. */
	if (prev_state != m_state) {
		struct movement_out_event *event = new_movement_out_event();
		event->state = m_state;
		/** @todo Add total_steps as well? */
		LOG_DBG("Total steps is %i, state is %i, activity is %i, acc_std_final %i",
			total_steps, m_state, cur_activity, acc_std_final);
		EVENT_SUBMIT(event);
		prev_state = m_state;
	}

	/* Reset the timer since we just consumed the data successfully. */
	k_timer_start(&movement_timeout_timer,
		      K_SECONDS(CONFIG_MOVEMENT_TIMEOUT_SEC), K_NO_WAIT);
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
		process_acc_data(&raw_data);
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
	data.x = (int16_t)(sensor_value_to_double(&accel[0]) * 1000);
	data.y = (int16_t)(sensor_value_to_double(&accel[1]) * 1000);
	data.z = (int16_t)(sensor_value_to_double(&accel[2]) * 1000);

	LOG_DBG("Acc X: %d, Y: %d, Z: %d", data.x, data.y, data.z);

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
	err = update_acc_odr_and_trigger(MODE_12_5_HZ);
	if (err) {
		return err;
	}

	/* Start the timeout timer. This is reset everytime we have calculated
	 * the activity successfully.
	 */
	k_timer_start(&movement_timeout_timer,
		      K_SECONDS(CONFIG_MOVEMENT_TIMEOUT_SEC), K_NO_WAIT);

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

EVENT_LISTENER(move_controller, event_handler);
EVENT_SUBSCRIBE(move_controller, movement_set_mode_event);