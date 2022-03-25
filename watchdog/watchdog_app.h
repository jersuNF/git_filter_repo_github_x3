/*
 * Copyright (c) 2021 Nofence AS
 */

/**@file
 *
 * @brief   Watchdog module
 */

#ifndef WATCHDOG_APP_H__
#define WATCHDOG_APP_H__

#include <zephyr.h>
#include "error_event.h"

/** @brief Initialize and start application watchdog module.
 *
 *  @return Zero on success, otherwise a negative error code is returned.
 */
int watchdog_init_and_start(void);

#endif /* WATCHDOG_APP_H__ */
