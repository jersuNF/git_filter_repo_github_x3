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
 * @brief Function to test the battery
 */
void fetch_periodic_battery_voltage(void);

#endif /* _PWR_MODULE_H_ */