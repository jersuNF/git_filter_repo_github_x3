/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include <logging/log.h>
#include <drivers/sensor.h>

#include "movement_controller.h"
#include "movement_events.h"
#include "stg_config.h"
#include "nf_fifo.h"
#include "trigonometry.h"

LOG_MODULE_REGISTER(move_controller, CONFIG_MOVE_CONTROLLER_LOG_LEVEL);

#define STEPS_TRIGGER 1
#define ACC_FIFO_ELEMENTS 32
#define SENSOR_SAMPLE_INTERVAL_MS 100

#define GRAVITY 9.806650

enum acc_reading_state {
	FIRST_READ_DISREGARD_BUFFER = 0,
	SECOND_READ_INIT_MOVING_AVERAGE,
	NORMAL_READ
};

static enum acc_reading_state reading_state = FIRST_READ_DISREGARD_BUFFER;

static const struct device *sensor;

/* +∕- fullscale range [g] */
typedef enum { RANGE_2G = 2, RANGE_4G = 4, RANGE_8G = 8, RANGE_16G = 16 } acc_scale_t;

/** @todo Add to make configurable from messaging.c event. Also storage settings? */
static uint16_t off_animal_time_limit_sec = OFF_ANIMAL_TIME_LIMIT_SEC_DEFAULT;
static uint16_t acc_sigma_sleep_limit = ACC_SIGMA_SLEEP_LIMIT_DEFAULT;
static uint16_t acc_sigma_noactivity_limit = ACC_SIGMA_NOACTIVITY_LIMIT_DEFAULT;

static int16_t acc_fifo_x[ACC_FIFO_ELEMENTS];
static int16_t acc_fifo_y[ACC_FIFO_ELEMENTS];
static int16_t acc_fifo_z[ACC_FIFO_ELEMENTS];

/* Save the calculated standard deviation for later. */
static uint32_t acc_std_final = 0;

static uint32_t first_inactive_timestamp = 0;
static uint16_t activity_decrease_timestamp = 0;
static uint32_t active_timestamp = 0;
static movement_state_t prev_state = STATE_INACTIVE;
static uint8_t num_acc_fifo_samples = 0;
static acc_activity_t last_activity = ACTIVITY_NO;

/* Work queue item for triggering a read from the sensor */
static struct k_work_delayable sample_sensor_work;

/* Variable used to check if we time out regarding new accelerometer data. */
void movement_timeout_fn(struct k_timer *dummy);
K_TIMER_DEFINE(movement_timeout_timer, movement_timeout_fn, NULL);

void movement_thread_fn();
K_THREAD_DEFINE(movement_thread, CONFIG_MOVEMENT_THREAD_STACK_SIZE, movement_thread_fn, NULL, NULL,
		NULL, CONFIG_MOVEMENT_THREAD_PRIORITY, 0, 0);

K_MSGQ_DEFINE(acc_data_msgq, sizeof(raw_acc_data_t), CONFIG_ACC_MSGQ_SIZE, 4);

/* Non-production code used to initialize persitstent variables to known state */
#ifdef CONFIG_TEST
raw_acc_data_t raw_data;
void _movement_controller_reset_for_test(void)
{
	reading_state = FIRST_READ_DISREGARD_BUFFER;
	acc_std_final = 0;
}
uint32_t _movement_controler_get_acc_std_final()
{
	return acc_std_final;
}
#endif

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
		acceleration[i] = -(acc_fifo_y[i] - gravity);
	}

	/* Count the number of steps in the FIFO-queue. */
	for (i = 0; i < ACC_FIFO_ELEMENTS; i++) {
		if (acceleration[i] < 0 && step_flag == false) {
			step_flag = true;
			acc_fifo_y[i] = gravity;
		}
		if (acceleration[i] > STEP_THRESHOLD && step_flag == true) {
			step_flag = false;
			step_counter++;
			acc_fifo_y[i] = gravity;
		}
	}
	return step_counter;
}

/**
 * @brief Processing of accelerometer data to compute animal activity level.
 *  
 * @details The animal activity computation is only performed on 32 consecutive 
 * samples from the accelerometer. Thus, this function fills the data buffer
 * from the first 31 calls of this function and perform the calculation on the 
 * 32 call to this function.
 * 
 * @param acc Accelerometer data in the X-,Y- and Z-axis (See raw_acc_data_t).
 */
void process_acc_data(raw_acc_data_t *acc)
{
	if (acc == NULL) {
		LOG_ERR("No data available.");
		return;
	}

	LOG_DBG("acc_std_final mov_controller: %d", acc_std_final);

	acc_fifo_x[num_acc_fifo_samples] = acc->x;
	acc_fifo_y[num_acc_fifo_samples] = acc->y;
	acc_fifo_z[num_acc_fifo_samples] = acc->z;
	num_acc_fifo_samples++;

	if (num_acc_fifo_samples < ACC_FIFO_ELEMENTS) {
		LOG_DBG("Filling data buffer, sample %d/%d", num_acc_fifo_samples,
			(ACC_FIFO_ELEMENTS - 1));
		return;
	}
	LOG_DBG("Accel. data acquired (%d/%d samples), processing", num_acc_fifo_samples,
		ACC_FIFO_ELEMENTS);

	num_acc_fifo_samples = 0;
	if (reading_state == FIRST_READ_DISREGARD_BUFFER) {
		reading_state = SECOND_READ_INIT_MOVING_AVERAGE;
		/* disregard the first 32 readings as they might contain MEMS garbage */
		LOG_DBG("Disregards first accelermoter data");
		return;
	}

	/* buffers are filled, start algorithm. */
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
		uint32_t acc_tot = (uint32_t)(acc_fifo_x[i] * acc_fifo_x[i]) +
				   (uint32_t)(acc_fifo_y[i] * acc_fifo_y[i]) +
				   (uint32_t)(acc_fifo_z[i] * acc_fifo_z[i]);
		acc_norm[i] = g_u32_SquareRootRounded(acc_tot);
		acc_norm_sum += acc_norm[i];
		gravity += acc_fifo_y[i] / ACC_FIFO_ELEMENTS;
	}
	uint32_t acc_norm_mean = acc_norm_sum / ACC_FIFO_ELEMENTS;

	/* Compute Standard Deviation. */
	uint32_t acc_std = 0;

	for (i = 0; i < ACC_FIFO_ELEMENTS; i++) {
		int32_t x = (acc_norm[i] - acc_norm_mean);
		acc_std += (uint32_t)(x * x);
	}
	acc_std /= ACC_FIFO_ELEMENTS;
	acc_std = g_u32_SquareRootRounded(acc_std);
	/* Exponential moving average with alpha = 2/(N+1). */
	if (reading_state == SECOND_READ_INIT_MOVING_AVERAGE) {
		reading_state = NORMAL_READ;
		acc_std_final = acc_std;
	} else {
		acc_std_final = (acc_std * 2) / (ACC_STD_EXP_MOVING_AVERAGE_N + 1) + acc_std_final -
				(acc_std_final * 2) / (ACC_STD_EXP_MOVING_AVERAGE_N + 1);
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
	struct activity_level *animal_activity = new_activity_level();
	animal_activity->level = cur_activity;
	EVENT_SUBMIT(animal_activity);

	LOG_DBG("Step count = %d", stepcount);

	if (stepcount >= STEPS_TRIGGER) {
		struct step_counter_event *steps = new_step_counter_event();
		steps->steps = stepcount;
		EVENT_SUBMIT(steps);
	}

	/* Gradually increase or decrease of activity level. 
     * If activity is greater than the last, increment instantly. 
	 */
	uint32_t a_delta = k_uptime_get_32() - activity_decrease_timestamp;
	if (cur_activity > last_activity) {
		cur_activity = last_activity + 1;
		activity_decrease_timestamp = k_uptime_get_32();
	} else if (cur_activity < last_activity) {
		if ((last_activity == ACTIVITY_LOW && a_delta > A_DEC_THRESHOLD_NO) ||
		    (last_activity == ACTIVITY_MED && a_delta > A_DEC_THRESHOLD_LOW) ||
		    (last_activity == ACTIVITY_HIGH && a_delta > A_DEC_THRESHOLD_MED)) {
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
			(uint32_t)((k_uptime_get_32() - first_inactive_timestamp) / 1000);
		if (inactive_for_sec >= off_animal_time_limit_sec) {
			is_active = false;
		}
	}

	/* Update current activity level. */
	last_activity = (acc_activity_t)cur_activity;

	/* Determine the movement state. */
	movement_state_t m_state = STATE_SLEEP;
	if (!is_active) {
		m_state = STATE_INACTIVE;
	} else {
		active_timestamp = k_uptime_get_32();
		if (acc_std_final >= acc_sigma_sleep_limit) {
			m_state = STATE_NORMAL;
		}
	}

	/* Submit activity mode and stepcount event if have a new state. */
	if (prev_state != m_state) {
		struct movement_out_event *event = new_movement_out_event();
		event->state = m_state;

		LOG_DBG("State is %i, activity is %i, acc_std_final %i", m_state, cur_activity,
			acc_std_final);
		EVENT_SUBMIT(event);
		prev_state = m_state;
	}

	/* Reset the timer since we just consumed the data successfully. */
	k_timer_start(&movement_timeout_timer, K_SECONDS(CONFIG_MOVEMENT_TIMEOUT_SEC), K_NO_WAIT);
}

uint32_t get_active_delta(void)
{
	return prev_state == STATE_INACTIVE ? 0 : k_uptime_get_32() - active_timestamp;
}

void movement_thread_fn()
{
	while (true) {
		raw_acc_data_t raw_data;
		int err = k_msgq_get(&acc_data_msgq, &raw_data, K_FOREVER);
		if (err) {
			LOG_ERR("Error retrieving accelerometer message queue %i", err);
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
	/* Convert from m/s² to mg */
	int32_t accel_mg[3];
	accel_mg[0] = (int32_t)((sensor_value_to_double(&accel[0]) / GRAVITY) * 1000);
	accel_mg[1] = (int32_t)((sensor_value_to_double(&accel[1]) / GRAVITY) * 1000);
	accel_mg[2] = (int32_t)((sensor_value_to_double(&accel[2]) / GRAVITY) * 1000);

	LOG_DBG("Acceleration [mg]: X: %d, Y: %d, Z: %d", accel_mg[0], accel_mg[1], accel_mg[2]);

	raw_acc_data_t data;
	data.x = (int16_t)(accel_mg[0] * 16); // Multiply with legacy constant
	data.y = (int16_t)(accel_mg[1] * 16); // Multiply with legacy constant
	data.z = (int16_t)(accel_mg[2] * 16); // Multiply with legacy constant

	LOG_DBG("Raw values:  X: %d, Y: %d, Z: %d", data.x, data.y, data.z);

	while (k_msgq_put(&acc_data_msgq, &data, K_NO_WAIT) != 0) {
		/* Message queue is full: purge old data & try again */
		LOG_WRN("Message queue full, purging and retry");
		k_msgq_purge(&acc_data_msgq);
	}

#ifdef CONFIG_TEST
	/* Keep a copy of the raw data for testing */
	memcpy(&raw_data, &data, sizeof(data));
#endif
}

static void sample_sensor_work_fn(struct k_work *work)
{
	k_work_reschedule(&sample_sensor_work, K_MSEC(SENSOR_SAMPLE_INTERVAL_MS));
	fetch_and_display(sensor);
}

static int update_acc_odr(acc_mode_t mode_hz)
{
	/* Setup interrupt triggers. */
	struct sensor_trigger trig;
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
#if CONFIG_LIS2DH
		hz = 10;
#else
		hz = 12;
#endif
		break;
	}
	default: {
		return -EINVAL;
	}
	}
	struct sensor_value odr = {
		.val1 = hz,
	};

	int ret = sensor_attr_set(sensor, trig.chan, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
	if (ret != 0) {
		LOG_ERR("Failed to set odr: %d", ret);
		return ret;
	}
	return 0;
}

static int update_acc_range(acc_scale_t scale)
{
	struct sensor_value range;
	sensor_g_to_ms2((uint8_t)scale, &range);
	int ret = sensor_attr_set(sensor, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &range);
	if (ret != 0) {
		LOG_ERR("Failed to set range: %d", ret);
		return ret;
	}
	return 0;
}

int init_movement_controller(void)
{
	int err;

	sensor = device_get_binding(DT_LABEL(DT_NODELABEL(movement_sensor)));

	if (sensor == NULL) {
		LOG_ERR("Could not find LIS2DW12 driver.");
		return -ENODEV;
	}
	if (!device_is_ready(sensor)) {
		LOG_ERR("Failed to setup LIS2DW12 accelerometer driver.");
		return -EFAULT;
	} else {
		LOG_INF("Setup LIS2DW12 accelerometer driver.");
	}

	/* Setup interrupt triggers. */
	err = update_acc_odr(MODE_12_5_HZ);
	if (err != 0) {
		return err;
	}

	/* NB: This overwrites the default range set in boardfile */
	err = update_acc_range(RANGE_2G);
	if (err != 0) {
		return err;
	}

	/* Init and start the sensor sampling work item */
	k_work_init_delayable(&sample_sensor_work, sample_sensor_work_fn);
	k_work_reschedule(&sample_sensor_work, K_MSEC(SENSOR_SAMPLE_INTERVAL_MS));

	/* Start the timeout timer. This is reset everytime we have calculated
	 * the activity successfully. */
	k_timer_start(&movement_timeout_timer, K_SECONDS(CONFIG_MOVEMENT_TIMEOUT_SEC), K_NO_WAIT);

	err = stg_config_u16_read(STG_U16_ACC_SIGMA_SLEEP_LIMIT, &acc_sigma_sleep_limit);
	if (err != 0) {
		return err;
	}

	err = stg_config_u16_read(STG_U16_ACC_SIGMA_NOACTIVITY_LIMIT, &acc_sigma_noactivity_limit);
	if (err != 0) {
		return err;
	}

	err = stg_config_u16_read(STG_U16_OFF_ANIMAL_TIME_LIMIT_SEC, &off_animal_time_limit_sec);
	if (err != 0) {
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
		struct movement_set_mode_event *ev = cast_movement_set_mode_event(eh);
		err = update_acc_odr(ev->acc_mode);
		if (err) {
			return false;
		}
		return false;
	}
	if (is_acc_sigma_event(eh)) {
		struct acc_sigma_event *ev = cast_acc_sigma_event(eh);
		switch (ev->type) {
		case SLEEP_SIGMA: {
			acc_sigma_sleep_limit = ev->param.sleep_sigma_value;
			break;
		}
		case NO_ACTIVITY_SIGMA: {
			acc_sigma_noactivity_limit = ev->param.no_activity_sigma;
			break;
		}
		case OFF_ANIMAL_SIGMA: {
			off_animal_time_limit_sec = ev->param.off_animal_value;
			break;
		}
		default: {
			break;
		}
		}

		return false;
	}
	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(move_controller, event_handler);
EVENT_SUBSCRIBE(move_controller, movement_set_mode_event);
EVENT_SUBSCRIBE(move_controller, acc_sigma_event);
