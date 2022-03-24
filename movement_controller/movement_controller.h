/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _MOVEMENT_CONTROLLER_H_
#define _MOVEMENT_CONTROLLER_H_

#include <zephyr.h>
#include "event_manager.h"

typedef struct {
	double x;
	double y;
	double z;
} raw_acc_data_t;

int init_movement_controller(void);

//#define RAW_FIFOSIZE 32
//
////#define DBG_WRITE_RAWFIFO
////#define DBG_WRITE_STEPQUEUE
////#define DBG_WRITE_ACTIVITYLEVEL
//
////#define ACC_MODE_OFF	0
//#define ACC_MODE_1HZ 1
//#define ACC_MODE_10HZ 2
////#define ACC_MODE_100HZ	3
//
////Sleep is when the collar is in ACTIVITY_NO for 10 or more minutes
//#define ACTIVITY_NO 0 // Totally steady position
//#define ACTIVITY_LOW 1 // No steps
//#define ACTIVITY_MED 2 // One step or more in the buffer
//#define ACTIVITY_HIGH 3 // STEPS steps or more
//
//#define ACTIVITY_DECREASE_THRESHOLD_NO                                         \
//	60 // seconds the device has to be in ACTIVITY_NO for it to go from ACTIVITY_LOW to ACTIVITY_NO
//#define ACTIVITY_DECREASE_THRESHOLD_LOW                                        \
//	15 //                                 ACTIVITY_LOW                  ACTIVITY_MED    ACTIVITY_LOW
//#define ACTIVITY_DECREASE_THRESHOLD_MED                                        \
//	0 //								   ACTIVITY_MED				     ACTIVITY_HIGH   ACTIVITY_MED
//
//#define STEP_THRESHOLD                                                         \
//	3000 // amplitude of z-acceleration needed to count a step
//#define STEPS                                                                  \
//	10 // number of steps needed in the step counter for activity to be HIGH
//
//#define STEP_QUEUE_TIME                                                        \
//	3600 // number of seconds every column in the step queue represent. When 3600, every column in the step queue count the steps for an hour
//#define STEP_QUEUE_LENGTH                                                      \
//	24 // number of columns in the step queue, when 24 and 3600, the step queue remembers the number of steps for the last 24 hours
//
//volatile uint16_t acc_GetInactiveTime(void);
//void acc_ResetActiveTime(void);
//volatile uint16_t acc_GetActiveTime(void);
//uint32_t acc_getAccumulatedSigma();
//
//// should only be called immediately before a sleep in order to wakeup
//void acc_clearFIFO();
//
//void acc_Init(void);
//
//uint8_t acc_ReadFIFO(void);
//
//// retrieves the number of steps over the last number of hours
//uint32_t acc_GetSteps(uint8_t hours);
//uint8_t acc_CountSteps(int16_t myFIFO[RAW_FIFOSIZE][3], int32_t myGravity);
//
//// retrieves the number of steps since last reset
//uint32_t acc_GetTotalSteps(void);
//void acc_ResetTotalSteps(uint32_t resetValue);
//
//uint8_t acc_ActivityLevel(void);

#endif /* _MOVEMENT_CONTROLLER_H_ */