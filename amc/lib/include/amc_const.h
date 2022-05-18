/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _AMC_CONST_H_
#define _AMC_CONST_H_

#include <zephyr.h>

/* FIFO constants. */
#define FIFO_ELEMENTS 8
#define FIFO_AVG_DISTANCE_ELEMENTS 15

/* How many zaps during a breakout before animal is considered 'escaped'. */
#define PAIN_CNT_DEF_ESCAPED 3

/* Warning duration before pain (sec) if the distance increase is very slow. 
 * 0 turns off function. 
 */
#define WARN_MAX_DURATION 20

/* Warning duration before pain (sec) if the distance increase is very fast. 
 * 0 turns off function. 
 */
#define WARN_MIN_DURATION 5

/* How many MS inbetween each buzzer update with current warn HZ. */
#define WARN_BUZZER_UPDATE_RATE 250

/* Time in sec every activity level has to be active 
 * before activity level is updated to a lower value. 
 */
#define ACTIVITY_UPDATERATE 30

/* Time between every tone update when tone increase/decrease [ms]. */
#define WARN_TONE_SPEED_MS                                                     \
	((WARN_FREQ_MAX - WARN_FREQ_INIT) / WARN_TONE_SPEED_HZ /               \
	 (WARN_MIN_DURATION_MS / 1000))

/* Steps of the tone update in hz. */
#define WARN_TONE_SPEED_HZ 11

/* Defined update rate (msec) on GPS at max "intensity" (5Hz). */
#define GPS_RATE_MAX 250

/* Default update rate (sec) when PSM mode. */
#define GPS_RATE_PSM_SEC 8

/* hDOP limit that is required to perform animal correction. */
#define HDOP_LIM 200

/* Min number of satellites that is required to perform animal correction. */
#define NUMSV_LIM 6

/* Height [dm] change is calculated. 
 * If the change during the last FIFO_ELEMENTS is greater than this value, 
 * the accuracy is set to bad fix.
 */
#define DELTAHEIGHT_LIM 20

/* Accuracy the last DIST_ELEMENTS has to be lower 
 * than this treshold for the warning to start 
 * (see http://youtrack.axbit.no/youtrack/issue/NOF-212).
 */
#define ACCDELTA_LIM 1

/* Course(heading) accuracy limit for start warn [deg*100]. */
#define CACC_LIM 36000

/* No height above this limit [dm]. */
#define HEIGHT_MAX_LIM 30000

/* No height below this limit  [dm]. */
#define HEIGHT_MIN_LIM ((int16_t)-10000)
/* The next warning sound after pain is this far away when pain [dm]. */
#define DIST_OFFSET_AFTER_PAIN 10
/* The next warning sound after Bad fix status is this far away from [dm]. */
#define DIST_OFFSET_AFTER_BADFIX 50

/* Minimum time the warning has to be off before new start (sec). */
#define CORRECTION_PAUSE_MIN_TIME 3

/* Seconds that is needed outside fence, without warning signal 
 * and not escaped status before Out of fence status to activate.
 */
#define OUT_OF_FENCE_TIME 900

// ---  Correction start/stop setup --- //

// Defines how many of the last DIST_ELEMENTS (8) that has to be greater than last distance for the warning to start
#define DIST_INCR_COUNT 0
#define TEACHMODE_DIST_INCR_COUNT 3

// The warning tone does not play up before the distance slope rate is higher than this, according to average of the last measurements
// Lower value -> easier to play tone upwards
#define DIST_INCR_SLOPE_LIM -8
#define TEACHMODE_DIST_INCR_SLOPE_LIM 0
#define TEACHMODE_DIST_DECR_SLOPE_OFF_LIM -6

// The warning plays down if the distance slope rate is lower than this, according to average of the last measurements
// Higher value -> easer to play tones downwards
#define DIST_DECR_SLOPE_LIM -9
#define TEACHMODE_DIST_DECR_SLOPE_LIM 0

/**
 * Distance in dm inside warning last started distance that is needed to pause warning in Fence Mode
 * E.g. if the warning started 4 m from the pasture border, and if the value is -20, the
 * collar has to move to 2 m from the pasture
 */
#define CORRECTION_PAUSE_DIST -20
#define TEACHMODE_CORRECTION_PAUSE_DIST -10

// Number of warnings that did not end in zap has to reach this limit before teachmode ends and FenceMode begins, 0 deactivates teachmode.
#define _TEACHMODE_WARN_CNT_LOWLIM 20

// If the dist history increases like this or more, the warning start easier. (half of the DIST_INCR_SLOPE_LIM and half of the DIST_INC_COUNT)
// Lower value -> easier to start warning
#define DIST_AVG_INCR_SLOPE_LIM 50

// Activity that defines reaction on both sound and shock
// Values is only valid when accelerometer is running at 100Hz
// range is 0-65
#define ACC_STOP_AMPLITUDE                                                     \
	50 // The accelerometer amplitude value has to be bigger than this for the sound warning to stop
#define ACC_SHOCK_AMPLITUDE                                                    \
	60 // The accelerometer amplitude value has to be more than this for the system to know that the animal felt the shock

// Be certain that the animal don't reach fence limit within LogInterval time!
// Dist lower than this sets GPS to Power save mode and calculates sleep duration based upon fence dist. This duration uses caution zone size to determent each 1sec interval

/**
 * Can only enter the PSMZone if accumulated distance to fence is less that this value
 */
#define LIM_CAUTION_LOW_DM ((int16_t)-120)
/**
 * Can only exit the PSMZone if accumulated distance to fence is higher that this value
 */
#define LIM_CAUTION_HIGH_DM ((int16_t)-100)

/**
 * Can only enter the CautionZone if distance to fence is less that this value
 */
#define LIM_PREWARN_LOW_DM ((int16_t)-70)
/**
 * Can only exit the CautionZone if distance to fence is higher that this value
 */
#define LIM_PREWARN_HIGH_DM ((int16_t)-50)

/**
 * No corrections can be started with a distance-to-pasture less than this value.
 * However, due to dynamic limits and GPS limitations, warnings can be started at
 * a greater distance.
 */
#define LIM_WARN_MIN_DM ((int16_t)0)

/**
 * Upper GNSS hAcc limit for correction (in dm). Position measurements with hAcc larger than this will be
 * invalidated for corrections. Note however that we use "dynamic" pasture borders, i.e. with a
 * worse hAcc, the collar has to be further away from the pasture border than for a good hAcc.
 * NB: NOF-548 reverses dynamic pastures, so we set this limit to 7 m again for HW_F
 */
#if defined(HW_F)
#define GNSS_HACC_UPPER_LIM_DM 70
#else
#define GNSS_HACC_UPPER_LIM_DM 35
#endif

#define ZAP_EVALUATION_TIME 200

#define ZAP_DURATION_0 300 /* 160. */
#define ZAP_DURATION_1 0 /* 250. */
#define ZAP_DURATION_2 300

/**
 * When warning ends (pauses) as a result of returning movement 
 * or orientation towards fence, the 
 * "new fence line" will be placed this distance further away [dm].
 * If 0, the fence already warned about will still be on the same distance,
 * but if 1 the warning tone will be Restarted at this very distance.
 */
#define _LAST_DIST_ADD 1

#endif /* _AMC_CONST_H_ */