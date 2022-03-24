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

/** @brief Resets the state of the zone calculator to initial value; NO_ZONE. 
*/
void zone_reset(void);

/** @brief Get current zone.
 * 
 * @returns Current zone. 
 */
amc_zone_t zone_get(void);

/** @brief Update the current zone by providing updated distance, 
 *         and whether or not position is valid. 
 * 
 * @param[in] instant_dist Distance from fence. 
 * @param[in] distance_is_valid Set to true if distance is valid, 
 *                              false otherwise.
 * 
 * @returns Updated zone. 
 */
amc_zone_t zone_update(int16_t instant_dist, bool distance_is_valid);

#endif /* _AMC_ZONE_H_ */