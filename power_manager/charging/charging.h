/*
 * Copyright (c) 2021 Nofence AS
 */

#ifndef _CHARGING_H_
#define _CHARGING_H_

#include <zephyr.h>

/**
 * @brief Init the charging module
 *
 * @return 0 on success, otherwise negative error code.
 */
int init_charging_module(void);

/**
 * @brief Start charging, set GPIO to enable transistor
 *
 * @return 0 on success, otherwise negative error code.
 */
int start_charging(void);

/**
 * @brief Stop charging, disable GPIO
 *
 * @return 0 on success, otherwise negative error code.
 */
int stop_charging(void);

/**
 * @brief Perform a current measurement
 *
 * @return 0 on success, otherwise negative error code.
 */
int read_analog_charging_channel(void);

/**
 * @brief Init the moving average struct for current measurements
 *
 */
void init_current_moving_average(void);

/**
 * @brief Fetch the averaged current measurement
 *
 * @return current measurement given in mA. Return -ENOENT if reading fails.
 */
int current_sample_averaged(void);

/** 
 * @brief Configure the solar panel charging adc
 * @return 0 on success, otherwise -ENOENT error code if device not ready.
 */
int charging_setup(void);

#endif /* _CHARGING_H_ */