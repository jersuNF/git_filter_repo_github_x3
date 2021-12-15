/*
 * Copyright (c) 2021 Nofence AS
 */

#ifndef _FW_UPGRADE_EVENTS_H_
#define _FW_UPGRADE_EVENTS_H_

#include "event_manager.h"
#include <zephyr.h>

/** @brief Enum for defining where the fragment came from, 
 *         or more specifically what triggered the DFU process.
 */
enum dfu_trigger_type {
	DFU_TRIGGER_TYPE_IDLE = 0,
	DFU_TRIGGER_TYPE_BLUETOOTH = 1,
	DFU_TRIGGER_TYPE_MODEM = 2
};

/** @brief Struct containg fragment data as well as a dyndata field
 *         that is used for dynamic fragment allocation and storage.
 */
struct dfu_fragment_event {
	struct event_header header;

	enum dfu_trigger_type trigger_type;
	size_t file_size;
	struct event_dyndata dyndata;
};

/** @brief Enum for different dfu statuses, 
 *         if we want modules to shutdown correctly etc...
 *         We call it schedule reboot, since fw_upgrade module must schedule 
 *         a reboot after n seconds when done.
 */
enum dfu_status_flag {
	DFU_STATUS_IDLE = 0,
	DFU_STATUS_IN_PROGRESS = 1,
	DFU_STATUS_SUCCESS_REBOOT_SCHEDULED = 2
};

/** @brief Struct containg status messages regarding the firmware upgrade. */
struct dfu_status_event {
	struct event_header header;

	enum dfu_trigger_type trigger_type;
	enum dfu_status_flag dfu_status;
};

EVENT_TYPE_DYNDATA_DECLARE(dfu_fragment_event);
EVENT_TYPE_DECLARE(dfu_status_event);

#endif /* _FW_UPGRADE_EVENTS_H_ */