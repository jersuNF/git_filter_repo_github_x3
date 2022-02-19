//
// Created by alaa on 18.02.2022.
//

#ifndef X3_FW_GPS_CONTROLLER_H
#define X3_FW_GPS_CONTROLLER_H

#include <zephyr.h>
#include "event_manager.h"

/**
 * @brief Used to initialize the gps_controller.
 *        Checks that the device is up and if required initializes its state
 *        through the driver's api calls.
 *
 * @return 0 on success, otherwise negative errno.
 */

int gps_controller_init(void);

typedef struct {
	.rate = 4;
	.power_save = true;
	.constellations = GALILEO | GPS | GLONAS;
}gnss_config;

#endif //X3_FW_GPS_CONTROLLER_H
