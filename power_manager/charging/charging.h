/*
 * Copyright (c) 2021 Nofence AS
 */

#ifndef _CHARGING_H_
#define _CHARGING_H_

#include <zephyr.h>

int init_charging_module(void);
int start_charging(void);
int stop_charging(void);

int read_analog_charging_channel(void);
int charging_current(void);
void init_current_moving_average(void);
int current_sample_averaged(void);

#endif /* _CHARGING_H_ */