/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _AMC_CORRECTION_H_
#define _AMC_CORRECTION_H_

#include <zephyr.h>
#include "embedded.pb.h"
#include "gnss.h"
#include "amc_zone.h"

/** @brief Performs the correction and starts, pauses the correction
 *         based on statuses. Also submits EP and sound events based on 
 *         parameters.
 * 
 * @param amc_mode Mode of the amc, i.e teach or not.
 * @param gnss last fix struct.
 * @param fs status of the fence (i.e animal location relative to pasture).
 * @param zone which zone we're currently in.
 * @param mean_dist mean distance to border.
 * @param dist_change distance change since previous calculation.
 * 
 * @returns 0 on success, otherwise negative errno.
 */
int process_correction(Mode amc_mode, gnss_last_fix_struct_t gnss,
		       FenceStatus fs, amc_zone_t zone, int16_t mean_dist,
		       int16_t dist_change);

#endif /* _AMC_CORRECTION_H_ */