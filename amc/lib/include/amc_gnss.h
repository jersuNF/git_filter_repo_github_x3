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

/** @brief Push new GNSS position update to
 * 
 * @param[in] timeout_cb Callback to call on timeout. 
 * 
 * @returns 0 on success, error code otherwise. 
 */
int gnss_update(gnss_struct_t* gnss_data);

#endif /* _AMC_GNSS_H_ */