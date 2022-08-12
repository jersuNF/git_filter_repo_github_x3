/*
 * Copyright (c) 2021 Nofence AS
 */

#ifndef _PWR_MODULE_H_
#define _PWR_MODULE_H_

#include <zephyr.h>
#include "event_manager.h"

enum pwr_requester_module {
	REQ_EP_MODULE = 0,
	REQ_SOUND_CONTROLLER = 1,
	REQ_END_OF_LIST
};

/**
 * @brief Initialize the power manager module.
 *        Set the corresponding GPIO pin.
 * @return 0 on success. Otherwise a negative error code.
 */
int pwr_module_init(void);

/**
 * @brief Returns the reboot reason for a soft reboot.
 * NB! The reboot reason is reset when read.
 * 
 * @param[out] aReason Reason for soft reset.
 * @return 0 on success, otherwise a negative error code.
 */
int pwr_module_reboot_reason(uint8_t *aReason);

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

/**
 * @brief Set whether module requires external clock to be active.
 * 
 * @param[in] req Requester id
 * @param[in] use_extclk Set to true when external clock is required.
 * 
 * @return 0 on success. Otherwise a negative error code.
 */
int pwr_module_use_extclk(enum pwr_requester_module req, bool use_extclk);

#endif /* _PWR_MODULE_H_ */
