/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include "movement_controller.h"
#include "movement_events.h"
#include <logging/log.h>

#define LOG_MODULE_NAME move_controller
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_MOVE_CONTROLLER_LOG_LEVEL);
