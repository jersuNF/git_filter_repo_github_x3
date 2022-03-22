/*
 * Copyright (c) 2022 Nofence AS
 */

#include <zephyr.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(amc_zone, CONFIG_AMC_LIB_LOG_LEVEL);

#include "amc_zone.h"

/** Markers indicate start distance for each zone, where increasing index
 *  indicates increasing level of zone where PSM_ZONE is lowest and
 *  WARN_ZONE is max. */
typedef struct {
	int16_t distance;
	int16_t hysteresis;
} zone_mark_t;

static const zone_mark_t markers[] = {
	 {.distance = INT16_MIN,
	 .hysteresis = 0}, 

	{.distance = CONFIG_ZONE_CAUTION_DIST, 
	 .hysteresis = CONFIG_ZONE_CAUTION_HYST},

	{.distance = CONFIG_ZONE_PREWARN_DIST, 
	 .hysteresis = CONFIG_ZONE_PREWARN_HYST},

	{.distance = CONFIG_ZONE_WARN_DIST, 
	 .hysteresis = CONFIG_ZONE_WARN_HYST},

	 {.distance = INT16_MAX,
	 .hysteresis = 0}
};

static amc_zone_t zone = NO_ZONE;

void zone_reset(void)
{
	zone = NO_ZONE;
}

/** @brief Calculates the zone based on distance and current zone. 
 * 
 * @param[in] current_zone Current zone. 
 * @param[in] instant_dist Distance from fence. 
 * 
 * @returns Calculated zone. 
 */
static amc_zone_t zone_calculate(amc_zone_t current_zone, int16_t instant_dist)
{
	amc_zone_t new_zone;

	/* Find zone for given distance */
	uint32_t markers_cnt = sizeof(markers)/sizeof(zone_mark_t);
	for (uint32_t i = 0; i < markers_cnt-1; i++) {
		new_zone = (i+PSM_ZONE);

		/* Distance must be within [start,end> to be within zone */
		int16_t start = markers[i].distance;
		int16_t end = markers[i+1].distance;
		
		/* Apply hysteresis depending on current zone */
		if (current_zone == new_zone) {
			start -= markers[i].hysteresis;
			end += markers[i+1].hysteresis;
		} else if (current_zone < new_zone) {
			start += markers[i].hysteresis;
			end += markers[i+1].hysteresis;
		} else {
			start -= markers[i].hysteresis;
			end -= markers[i+1].hysteresis;
		}

		if ((instant_dist >= start) && (instant_dist < end)) {
			break;
		}
	}
	
	return new_zone;
}

amc_zone_t zone_get(void)
{
	return zone;
}

amc_zone_t zone_update(int16_t instant_dist, bool distance_is_valid)
{
	if (distance_is_valid) {
		zone = zone_calculate(zone, instant_dist);
	} else {
		zone = NO_ZONE;
	}

	return zone;
}