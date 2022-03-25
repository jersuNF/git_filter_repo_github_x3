/*
 * Copyright (c) 2022 Nofence AS
 */

#include <zephyr.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(amc_states, CONFIG_AMC_LIB_LOG_LEVEL);

#include "amc_states.h"
#include "embedded.pb.h"

Mode get_mode(void)
{
	return Mode_Teach;
}

FenceStatus get_fence_status(void)
{
	return FenceStatus_Escaped;
}

CollarStatus get_collar_status(void)
{
	return CollarStatus_Stuck;
}

int set_sensor_modes(Mode amc_mode, gnss_mode_t gnss_mode, FenceStatus fs,
		     CollarStatus cs)
{
	/* Set STUFF!!!! */
	return 0;
}