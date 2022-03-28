/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _AMC_ZONE_H_
#define _AMC_ZONE_H_

#include <zephyr.h>

#include <collar_protocol.h>

typedef enum {
	/* Collar is in a state where zone information is irrelevant */
	NO_ZONE = 0,

	/* Collar is well inside fence borders */
	PSM_ZONE = 1,

	/* Collar is inside fence borders, but tolerance for position inaccuracies is lower */
	CAUTION_ZONE = 2,

	/* Collar is barely inside fence borders */
	PREWARN_ZONE = 3,

	/* Collar is outside of fence borders */
	WARN_ZONE = 4
} amc_zone_t;

/** @brief Get current zone.
 * 
 * @returns Current zone. 
 */
amc_zone_t zone_get(void);

/** @brief Set current zone. 
 * 
 * @param[in] new_zone Value of zone.
 * 
 * @returns 0 if ok, error code otherwise.
 */
int zone_set(amc_zone_t new_zone);

/** @brief Calculates the zone based on distance and current zone.
 *         Internal zone variable is set and value returned.
 * 
 * @param[in] instant_dist Distance from fence. 
 * 
 * @returns Calculated zone. 
 */
amc_zone_t zone_update(int16_t instant_dist);

/** @brief Get the time since last zone change. 
 * 
 * @returns Time since last zone change in milliseconds.
 */
uint64_t zone_get_time_since_update(void);

#endif /* _AMC_ZONE_H_ */