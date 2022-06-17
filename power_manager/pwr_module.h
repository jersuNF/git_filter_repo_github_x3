/*
 * Copyright (c) 2021 Nofence AS
 */

#ifndef _PWR_MODULE_H_
#define _PWR_MODULE_H_

#include <zephyr.h>
#include "event_manager.h"

/**
 * @brief Initialize the power manager module.
 *        Set the corresponding GPIO pin.
 * @return 0 on success. Otherwise a negative error code.
 */
int pwr_module_init(void);

/**
 * @brief Log battery voltage and battery precentage once.
 * @return battery voltage in mV
 * @return -ENOENT if no device is found, or if battery is not initialized.
 */
int log_and_fetch_battery_voltage(void);

/**
 * @brief Fetches the battery percent.
 * @return battery percent.
 */
int fetch_battery_percent(void);

#endif /* _PWR_MODULE_H_ */