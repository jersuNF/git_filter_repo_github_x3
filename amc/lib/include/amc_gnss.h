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

/** @brief Set GNSS power mode. 
 * 
 * @param[in] mode Power mode value to set. 
 * 
 * @returns 0 on success, error code otherwise. 
 */
int gnss_set_mode(gnss_mode_t mode);

/** @brief Get GNSS power mode. 
 * 
 * @returns Power mode of GNSS
 */
gnss_mode_t gnss_get_mode(void);

/** @brief Validate that GNSS fix is good enough, and update flags
 * 
 * @param[in] mode Collar mode value.
 * @param[in] gnss_data Position data from GNSS
 * 
 * @returns 0 on success, error code otherwise. 
 */
int gnss_update(gnss_t* gnss_data);

/** @brief Check if GNSS has fix
 * 
 * @returns True if GNSS has fix, false otherwise.
 */
bool gnss_has_fix(void);

/** @brief Check if GNSS has accepted fix
 * 
 * @returns True if GNSS has accepted fix, false otherwise.
 */
bool gnss_has_accepted_fix(void);

/** @brief Check if GNSS has easy fix
 * 
 * @returns True if GNSS has easy fix, false otherwise.
 */
bool gnss_has_easy_fix(void);

/** @brief Check if GNSS has warn fix
 * 
 * @returns True if GNSS has warn fix, false otherwise.
 */
bool gnss_has_warn_fix(void);

#endif /* _AMC_GNSS_H_ */