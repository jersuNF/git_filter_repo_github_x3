/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _AMC_STATES_H_
#define _AMC_STATES_H_

#include <zephyr.h>
#include "embedded.pb.h"
#include "gnss.h"

#include "ble_beacon_event.h"

/** @todo Move these to another place? Should AMC definitions have a common place?
 * There will not be performed more than this number of shocks each day, 
 * tracemode if this number of shocks.
 */
#define MAX_PAIN_ONE_DAY UINT16_MAX
/* Number of zaps can not get higher than this during teachmode. */
#define _TEACHMODE_ZAP_CNT_HIGHLIM MAX_PAIN_ONE_DAY

/* Least needed zap count. */
#define TEACHMODE_ZAP_CNT_LOWLIM 0

/* Number of warnings that did not end in zap 
 * has to reach this limit before teachmode ends and FenceMode begins, 
 * 0 deactivates teachmode.
 */
#define _TEACHMODE_WARN_CNT_LOWLIM 20

/* Seconds that is needed outside fence, without warning signal 
 * and not escaped status before Out of fence status to activate.
 */
#define OUT_OF_FENCE_TIME 900

/* Distance of freedom. if set to 0, the animal will 
 * be defined as free if these conditions. 
 */
#define _FREEDOM_DIST 100

/* Max number of shocks before escaped status. */
#define _PAIN_CNT_DEF_ESCAPED 3

/** @brief Checks if we're in teach mode and sets the cache variables
 *         that is necessary.
 */
void init_mode_status(void);

/** @brief Calculates the amc mode based on internal zap/warn count variables.
 * 
 * @note These should be no need for a raw get_mode function, since we
 *       always want to check the conditions within this function when we
 *       fetch the mode. I.e we always want the newest, most relevant mode.
 * 
 * @returns The new, calculated mode.
 */
Mode calc_mode(void);

/** @brief Just returns current mode on RAM.
 * 
 * @returns Status that we're currently in.
 */
Mode get_mode(void);

/** @brief Calculates and gives the current fencestatus.
 * 
 * @param WIP.
 * 
 * @returns Mode that we're currently in.
 */
FenceStatus calc_fence_status(uint32_t maybe_out_of_fence_timestamp);

/** @brief Calculates and gives the current collarstatus.
 * 
 * @param WIP.
 * 
 * @returns Mode that we're currently in.
 */
CollarStatus calc_collar_status(void);

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

/** @todo Should zap stuff be moved somewhere else? */
void increment_zap_count(void);
void increment_warn_count(void);

/** @brief Resets the pain zap count to 0. This is so that we can measure
 *         the number of zaps this breakout. I.e this is called on correction
 *         end.
 */
void reset_zap_pain_cnt(void);

/** @brief Sets the beacon status internally in amc_states that is used to
 *         determine the fence status.Is called from event_handler from
 *         beacon_status event.
 * 
 * @param status beacon status to update to.
 */
void set_beacon_status(enum beacon_status_type status);

#endif /* _AMC_STATES_H_ */