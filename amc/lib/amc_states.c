/*
 * Copyright (c) 2022 Nofence AS
 */

#include <zephyr.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(amc_states, CONFIG_AMC_LIB_LOG_LEVEL);

#include "amc_states.h"
#include "embedded.pb.h"

Mode get_amc_mode(void)
{
	return Mode_Teach;
}