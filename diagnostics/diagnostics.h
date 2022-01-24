/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _DIAGNOSTICS_H_
#define _DIAGNOSTICS_H_

#include <zephyr.h>
#include "event_manager.h"

#include "SEGGER_RTT_Conf.h"
#include "SEGGER_RTT.h"

#if (CONFIG_DIAGNOSTICS_RTT_DOWN_CHANNEL_INDEX > SEGGER_RTT_MAX_NUM_DOWN_BUFFERS)
#error "Diagnostics controller RTT down channel index is out of range."
#endif

#if (CONFIG_DIAGNOSTICS_RTT_UP_CHANNEL_INDEX > SEGGER_RTT_MAX_NUM_UP_BUFFERS)
#error "Diagnostics controller RTT up channel index is out of range."
#endif

enum diagnostics_severity {
	DIAGNOSTICS_INFO,
	DIAGNOSTICS_WARNING,
	DIAGNOSTICS_ERROR,
};

enum diagnostics_interface {
	DIAGNOSTICS_ALL,
	DIAGNOSTICS_RTT,
	DIAGNOSTICS_BLE,
	DIAGNOSTICS_NONE
	
};

/**
 * @brief Used to initilize the diagnostics module. 
 *        TODO - Add more...
 * 
 * @return 0 on success, otherwise negative errno.
 */
int diagnostics_module_init();

#endif /* _DIAGNOSTICS_H_ */