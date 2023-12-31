/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _MOVEMENT_CONTROLLER_H_
#define _MOVEMENT_CONTROLLER_H_

#include <zephyr.h>
#include "event_manager.h"
#include "stg_config.h"

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} raw_acc_data_t;

int init_movement_controller(void);

/** Seconds the device has to be in an activity to change to another activity. */
#define A_DEC_THRESHOLD_NO 60
#define A_DEC_THRESHOLD_LOW 15
#define A_DEC_THRESHOLD_MED 0

/** Amplitude of z-acceleration needed to count a step. */
#define STEP_THRESHOLD 1500

/** Number of steps needed in the step counter for activity to be HIGH. */
#define STEPS 10

/** Number of seconds every column in the step queue represent.
 *  When 3600, every column in the step queue count the steps for an hour.
 */
#define STEP_QUEUE_TIME 3600

/** Number of columns in the step queue, when 24 and 3600,
 *  the step queue remembers the number of steps for the last 24 hours.
 */
#define STEP_QUEUE_LENGTH 24

/**
 * Default Accelerometer limit for 2020 sleep detection algorithm.
 */
#define ACC_SIGMA_SLEEP_LIMIT_DEFAULT 600

/**
 * Default Accelerometer limit for how long it has to be inactive to be off. (30 * 60)
 */
#define OFF_ANIMAL_TIME_LIMIT_SEC_DEFAULT (30 * 60)

/**
 *  Default Accelerometer limit for 2021 No Activity detection algorithm.
 */
#define ACC_SIGMA_NOACTIVITY_LIMIT_DEFAULT 400

/**
 * Number of 3.2 seconds moving average samples for exponential moving average window size
 * used to implement Vegard Floviks sleep detection algorithm
 */
#define ACC_STD_EXP_MOVING_AVERAGE_N 60

/** @brief Used to get the delta of how long we've been in active state. */
uint32_t get_active_delta(void);

#ifdef CONFIG_TEST
void _movement_controller_reset_for_test(void);
uint32_t _movement_controler_get_acc_std_final(void);
#endif

#endif /* _MOVEMENT_CONTROLLER_H_ */