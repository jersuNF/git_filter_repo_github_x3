#ifndef X3_FW_CELLULAR_CONTROLLER_H
#define X3_FW_CELLULAR_CONTROLLER_H

#include <zephyr.h>
#include "event_manager.h"

/**
 * @brief Used to initialize the cellular_controller.
 *        Checks that the LTE network interface and configuration are ok
 *        and that the cellular connection is up, to start the tcp
 *        connection.
 *
 * @return 0 on success, otherwise negative errno.
 */

int8_t cellular_controller_init(void);

/**
 * @brief Check whether to network connection is up. 
 *
 * @return True if ready, false otherwise.
 */
bool cellular_controller_is_ready(void);

#endif //X3_FW_CELLULAR_CONTROLLER_H
