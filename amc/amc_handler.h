/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _AMC_HANDLER_H_
#define _AMC_HANDLER_H_

#include <zephyr.h>

/** @brief Initializes the work calculation thread and puts
 *         request events for fence and GNSS data.
 */
void amc_module_init(void);

#endif /* _AMC_HANDLER_H_ */