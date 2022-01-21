/*
 * Copyright (c) 2021 Nofence AS
 */

#ifndef _EP_MODULE_H_
#define _EP_MODULE_H_

#include <zephyr.h>
#include "event_manager.h"

/**
 * @brief Initialize the electrical pulse module.
 *        Set the corresponding GPIO pin.
 * @return 0 on success. Otherwise a negative error code.
 */
int ep_module_init(void);

/**
 * @brief Function to enable/disable the electrical pulse
 * @param[in] active Boolean true/false if electrical pulse is activ
 * @return 0 on success. Otherwise a negative error code.
 */
int ep_module_enable(bool active);

#endif /* _EP_MODULE_H_ */