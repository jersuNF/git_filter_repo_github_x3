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
int charging_init_module(void);

/**
 * @brief Start charging, set GPIO to enable transistor
 *
 * @return 0 on success, otherwise negative error code.
 */
int charging_start(void);

/**
 * @brief Stop charging, disable GPIO
 *
 * @return 0 on success, otherwise negative error code.
 */
int charging_stop(void);

/**
 * @brief Perform a current measurement
 *
 * @return 0 on success, otherwise negative error code.
 */
int charging_read_analog_channel(void);

/**
 * @brief Init the moving average struct for current measurements
 *
 */
void charging_init_moving_average(void);

/**
 * @brief Fetch the averaged current measurement
 *
 * @return current measurement given in mA. Return -ENOENT if reading fails.
 */
int charging_current_sample_averaged(void);

/** 
 * @brief Configure the solar panel charging adc
 * @return 0 on success, otherwise -ENOENT error code if device not ready.
 */
int charging_setup(void);

/** 
 * @brief Check if charging is running
 * @return true if charging is enabled, false otherwise
 */
bool charging_in_progress(void);

#endif /* _CHARGING_H_ */