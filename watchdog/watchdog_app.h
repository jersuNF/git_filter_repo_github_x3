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

/** @brief Initialize and start application watchdog module.
 *
 *  @return Zero on success, otherwise a negative error code is returned.
 */
int watchdog_init_and_start(void);

/** @brief Schedules a watchdog feed on the main system thread.
 *
 *  @warning Should only be used by error handler when we know all modules
 *           are alive and fully operational.
 */
void external_watchdog_feed(void);

#endif /* WATCHDOG_APP_H__ */
