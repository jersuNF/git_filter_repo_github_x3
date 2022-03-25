/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _AMC_STATES_H_
#define _AMC_STATES_H_

#include <zephyr.h>
#include "embedded.pb.h"

/** @brief Fetches the current amc mode.
 * 
 * @returns Mode that we're currently in.
 */
Mode get_amc_mode(void);

#endif /* _AMC_STATES_H_ */