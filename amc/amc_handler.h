/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _AMC_HANDLER_H_
#define _AMC_HANDLER_H_

#include <zephyr.h>

/** @brief Initializes the work calculation thread and puts
 *         request events for fence and GNSS data.
 */
void amc_module_init(void);

/**
 * @brief Function to request fence data on the event bus, in which case the
 *        storage module will memcpy its data to the passed address. This is
 *        only done when we boot up/on initialization, and on user updates
 *        in which case speed does not matter, as well as not
 *        needing the continuity as the GNSS data does.
 */
void submit_request_fencedata(void);

/**
 * @brief Function to request GNSS data on the event bus, in which case the
 *        GNSS module will memcpy its data to the passed address pointer.
 *        Up for discussion because it might be too slow, and perhaps could
 *        be better solutions for this continous/periodic data transfer.
 */
void submit_request_gnssdata(void);

#endif /* _AMC_HANDLER_H_ */