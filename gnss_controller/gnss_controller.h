
#ifndef X3_FW_GNSS_CONTROLLER_H
#define X3_FW_GNSS_CONTROLLER_H

#include <zephyr.h>
#include "event_manager.h"

/**
 * @brief Used to initialize the gps_controller.
 *        Checks that the device is up and if required initializes its state
 *        through the driver's api calls.
 *
 * @return 0 on success, otherwise negative errno.
 */

int gnss_controller_init(void);

#endif //X3_FW_GNSS_CONTROLLER_H
