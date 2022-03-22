/*
 * Copyright (c) 2022 Nofence AS
 */

#include <zephyr.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(amc_zone, CONFIG_AMC_LIB_LOG_LEVEL);

#include "amc_cache.h"
#include "amc_zone.h"

/*
const Mode mode = EEPROM_GetMode();
const CollarStatus collarStatus = EEPROM_GetCollarStatus();
const FenceStatus fenceStatus = EEPROM_GetFenceStatus();
uint8_t myGpsMode = GPSMODE_CAUTION;
*/

typedef struct {
	int16_t distance;
	int16_t hysteresis;
	amc_zone_t zone;
} zone_mark_t;

static const zone_mark_t markers[] = {
	{.distance = CONFIG_ZONE_CAUTION_DIST, 
	 .hysteresis = CONFIG_ZONE_CAUTION_HYST, 
	 .zone = CAUTION_ZONE},
	{.distance = CONFIG_ZONE_PREWARN_DIST, 
	 .hysteresis = CONFIG_ZONE_PREWARN_HYST, 
	 .zone = PREWARN_ZONE},
	{.distance = CONFIG_ZONE_WARN_DIST, 
	 .hysteresis = CONFIG_ZONE_WARN_HYST, 
	 .zone = WARN_ZONE},
};

static amc_zone_t zone = NO_ZONE;

void zone_reset(void)
{
	zone = NO_ZONE;
}

amc_zone_t zone_calculate(amc_zone_t current_zone, int16_t instant_dist)
{
	/* Zone is PSM zone unless found to be within another zone below */
	amc_zone_t new_zone = PSM_ZONE;

	/* Loop through all zone markers */
	for (uint32_t i = 0; i < sizeof(markers)/sizeof(zone_mark_t); i++) {
		/* Calculate threshold, with hysteresis based on current zone */
		int16_t threshold = markers[i].distance;
		if (current_zone < markers[i].zone) {
			threshold += markers[i].hysteresis;
		} else {
			threshold -= markers[i].hysteresis;
		}
		
		/* The zone has been found when distance is lower than threshold */
		if (instant_dist < threshold) {
			break;
		} else {
			new_zone = markers[i].zone;
		}
	}

	return new_zone;
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