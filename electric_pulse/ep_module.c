/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include <logging/log.h>

#include "ep_module.h"

#define LOG_MODULE_NAME ep_module
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_FW_UPGRADE_LOG_LEVEL);

int ep_module_init(void)
{
}