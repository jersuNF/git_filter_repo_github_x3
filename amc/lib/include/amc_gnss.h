/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _AMC_GNSS_H
#define _AMC_GNSS_H_

#include <zephyr.h>

#include "gnss.h"

#define GNSS_1SEC 1000
#define GNSS_5SEC (GNSS_1SEC * 5)
#define GNSS_10SEC (GNSS_1SEC * 10)
#define GNSS_20SEC (GNSS_1SEC * 20)

typedef int (*gnss_timeout_cb)(void);

/** @brief Initialize GNSS validator
 * 
 * @param[in] timeout_cb Callback to call on timeout. 
 * 
 * @returns 0 on success, error code otherwise. 
 */
int gnss_init(gnss_timeout_cb timeout_cb);

/** @brief Set GNSS power mode by submitting an event to GNSS controller.
 * 
 * @param[in] mode Power mode value to set. 
 * 
 * @returns 0 on success, error code otherwise. 
 */
int gnss_update_mode(gnss_mode_t mode);

/** @brief Get GNSS power mode. 
 * 
 * @returns Power mode of GNSS
 */
gnss_mode_t gnss_get_mode(void);

/** @brief Validate that GNSS fix is good enough, and update flags
 * 
 * @param[in] gnss_data Position data from GNSS
 * 
 * @returns 0 on success, error code otherwise. 
 */
int gnss_update(gnss_t *gnss_data);

/** @brief Convert latitude&longitude into X&Y coordinates based on origins
 * 
 * @param[in] gnss_data Position data from GNSS
 * @param[out] x_dm Pointer to variable to update with X coordinate
 * @param[out] y_dm Pointer to variable to update with Y coordinate
 * @param[in] origin_lon Longitude origin
 * @param[in] origin_lat Latitude origin
 * @param[in] k_lon longitude parameter
 * @param[in] k_lat Latitude parameter
 * 
 * @returns 0 on success, error code otherwise. 
 */
int gnss_calc_xy(gnss_t *gnss_data, int16_t *x_dm, int16_t *y_dm,
		 int32_t origin_lon, int32_t origin_lat, uint16_t k_lon,
		 uint16_t k_lat);

/** @brief Validate that GNSS fix is as good as possible and update flags.
 * 
 * @param[in] dist_avg_change Average distance change.
 * 
 * @param[in] dist_change Slope of previous distances.
 * @param[in] dist_incr_slope_lim Slope limit for dist_change.
 * 
 * @param[in] dist_inc_count Count for increasing elements in distance fifo.
 * @param[in] dist_incr_count Limit of increasing elements in distance fifo.
 * 
 * @param[in] height_delta MAX(height_arr) - MIN(height_arr).
 * @param[in] acc_delta MAX(acc_arr) - MIN(acc_arr)
 * 
 * @param[in] mean_dist Mean distance of distance array.
 * @param[in] h_acc_dm height accuracy of GNSS data.
 * 
 * @returns 0 on success, error code otherwise. 
 */
int gnss_update_dist_flags(int16_t dist_avg_change, int16_t dist_change,
			   int16_t dist_incr_slope_lim, uint8_t dist_inc_count,
			   uint8_t dist_incr_count, int16_t height_delta,
			   int16_t acc_delta, int16_t mean_dist,
			   uint16_t h_acc_dm);

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