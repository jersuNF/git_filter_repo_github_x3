/*
 * Copyright (c) 2021 Nofence AS
 */

/**@file
 *
 * @brief   Watchdog module
 */

#ifndef NOFENCE_WATCHDOG_H
#define NOFENCE_WATCHDOG_H

#include <zephyr.h>

/** @brief Initialize special nofence watchdog module
 *
 */
void nofence_wdt_init();

/** @brief Register a callback to be called when watchdogs triggers
 * 
 * @param cb callback pointer to register
*/
void nofence_wdt_register_cb(void (*cb)(uint8_t));

/** @brief register a module to the nofence watchdog module
 * 
 * @param module module name to register
 * @param wdt_timer_s watchdog trigger time in seconds 
 * @param reason reset reason to log
 * 
 * @return 0 on success
 * 
 * If a module calls this function and is already registered, it will
 * just udpate the trigger time
*/
int nofence_wdt_module_register(const char *module, uint8_t reason, uint32_t wdt_timer_s);

/** @brief register a module to the nofence watchdog module
 * 
 * @param module module name to register
 * @param wdt_timer_s watchdog trigger time in seconds 
 * 
 * @return 0 on success
*/
int nofence_wdt_module_unregister(const char *module);

/** @brief kick wdt for that module
 * 
 * @param module module name to kick
 * 
 * @return 0 on success
*/
int nofence_wdt_kick(const char *module);
#endif /* NOFENCE_WATCHDOG_H */
