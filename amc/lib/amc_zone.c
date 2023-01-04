/*
 * Copyright (c) 2022 Nofence AS
 */

#include <zephyr.h>
#include <logging/log.h>
#include <amc_events.h>
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
	{ .distance = INT16_MIN, .hysteresis = 0 },

	{ .distance = CONFIG_ZONE_CAUTION_DIST, .hysteresis = CONFIG_ZONE_CAUTION_HYST },

	{ .distance = CONFIG_ZONE_PREWARN_DIST, .hysteresis = CONFIG_ZONE_PREWARN_HYST },

	{ .distance = CONFIG_ZONE_WARN_DIST, .hysteresis = CONFIG_ZONE_WARN_HYST },

	{ .distance = INT16_MAX, .hysteresis = 0 }
};

static amc_zone_t zone = NO_ZONE;

static uint64_t zone_updated_at = 0;

int zone_update(int16_t instant_dist, gnss_t *gnss_data, amc_zone_t *updated_zone)
{
	int ret = 0;
	amc_zone_t new_zone;

	if ((gnss_data->latest.msss > CONFIG_ZONE_MIN_TIME_SINCE_RESET) &&
	    ((zone != PSM_ZONE) || (zone_get_time_since_update() > CONFIG_ZONE_PSM_LEAST_TIME))) {
		/* Find zone for given distance */
		uint32_t markers_cnt = sizeof(markers) / sizeof(zone_mark_t);
		for (uint32_t i = 0; i < markers_cnt - 1; i++) {
			new_zone = (i + PSM_ZONE);

			/* Distance must be within [start,end> to be within zone */
			int16_t start = markers[i].distance;
			int16_t end = markers[i + 1].distance;

			/* Apply hysteresis depending on current zone */
			if (zone == new_zone) {
				start -= markers[i].hysteresis;
				end += markers[i + 1].hysteresis;
			} else if (zone < new_zone) {
				start += markers[i].hysteresis;
				end += markers[i + 1].hysteresis;
			} else {
				start -= markers[i].hysteresis;
				end -= markers[i + 1].hysteresis;
			}

			if ((instant_dist >= start) && (instant_dist < end)) {
				break;
			}
		}

		zone_set(new_zone);
	} else {
		ret = -ETIME;

		if ((zone_get_time_since_update() > CONFIG_ZONE_LEAST_TIME) &&
		    (gnss_data->latest.msss > CONFIG_ZONE_MIN_TIME_SINCE_RESET) &&
		    ((zone != PSM_ZONE) || ((k_uptime_get_32() - gnss_data->latest.updated_at) >
					    CONFIG_MAX_PSM_FIX_AGE))) {
			zone_set(NO_ZONE);
		}
	}

	*updated_zone = zone;
	return ret;
}

amc_zone_t zone_get(void)
{
	return zone;
}

int zone_set(amc_zone_t new_zone)
{
	if (new_zone > WARN_ZONE) {
		return -EINVAL;
	}

	if (zone != new_zone) {
		zone = new_zone;
		zone_updated_at = k_uptime_get();

		struct zone_change *evt = new_zone_change();
		evt->zone = zone;
		EVENT_SUBMIT(evt);
	}
	return 0;
}

uint64_t zone_get_time_since_update(void)
{
	uint64_t time_since_update;

	time_since_update = k_uptime_get() - zone_updated_at;

	return time_since_update;
}
