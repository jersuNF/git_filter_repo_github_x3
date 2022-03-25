/*
 * Copyright (c) 2022 Nofence AS
 */

#include <zephyr.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(amc_correction, CONFIG_AMC_LIB_LOG_LEVEL);

#include "amc_correction.h"

static int correction_start()
{
	return 0;
}

static int correction_pause()
{
	return 0;
}

static int correction()
{
	return 0;
}

int process_correction(Mode amc_mode, gnss_last_fix_struct_t gnss,
		       FenceStatus fs, amc_zone_t zone, int16_t mean_dist,
		       int16_t dist_change)
{
	correction_start();
	correction_pause();
	correction();

	return 0;
}