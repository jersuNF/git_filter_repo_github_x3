/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _AMC_GNSS_H
#define _AMC_GNSS_H_

#include <zephyr.h>

#include "gnss.h"

typedef int (*gnss_timeout_cb)(void);

/** @brief Initialize GNSS validator
 * 
 * @param[in] timeout_cb Callback to call on timeout. 
 * 
 * @returns 0 on success, error code otherwise. 
 */
int gnss_init(gnss_timeout_cb timeout_cb);

/** @brief Validate that GNSS fix is good enough, and update timeout
 * 
 * @param[in] gnss_data Position data from GNSS
 * 
 * @returns 0 on success, error code otherwise. 
 */
int gnss_validate_and_update(gnss_t* gnss_data);

#endif /* _AMC_GNSS_H_ */