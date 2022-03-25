/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _AMC_STATES_H_
#define _AMC_STATES_H_

#include <zephyr.h>
#include "embedded.pb.h"
#include "gnss.h"

/** @brief Fetches the current amc mode.
 * 
 * @param WIP.
 * 
 * @returns Mode that we're currently in.
 */
Mode get_mode(void);

/** @brief Calculates and gives the current fencestatus.
 * 
 * @param WIP.
 * 
 * @returns Mode that we're currently in.
 */
FenceStatus get_fence_status(void);

/** @brief Calculates and gives the current collarstatus.
 * 
 * @param WIP.
 * 
 * @returns Mode that we're currently in.
 */
CollarStatus get_collar_status(void);

/** @brief Sets the sensor mode for those modules required.
 * 
 * @param amc_mode Mode of the amc, i.e teach or not.
 * @param gnss_mode mode of the gnss
 * @param fs status of the fence (i.e animal location relative to pasture)
 * @param cs status of the collar (i.e animal sleeping etc...)
 * 
 * @returns 0 on success, otherwise negative errno.
 */
int set_sensor_modes(Mode amc_mode, gnss_mode_t gnss_mode, FenceStatus fs,
		     CollarStatus cs);

#endif /* _AMC_STATES_H_ */