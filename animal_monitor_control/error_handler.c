/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include "error_event.h"
#include <logging/log.h>

#define LOG_MODULE_NAME error_handler
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_ERROR_HANDLER_LOG_LEVEL);
