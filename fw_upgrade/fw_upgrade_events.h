#ifndef _FW_UPGRADE_EVENTS_H_
#define _FW_UPGRADE_EVENTS_H_

#include <event_manager.h>

/* Enum for defining where the fragment came from, or more specifically what triggered the DFU process */
enum dfu_trigger_type {
	DFU_TRIGGER_TYPE_IDLE = 0,
	DFU_TRIGGER_TYPE_BLUETOOTH = 1,
	DFU_TRIGGER_TYPE_MODEM = 2
};

struct dfu_fragment_event {
	struct event_header header;

	/* Fragment payload and metadata */
	struct event_dyndata fragment;
	enum dfu_trigger_type trigger_type;
    size_t fragment_size;
	size_t file_size;
};

/* Enum for different dfu statuses, if we want modules to shutdown correctly etc...
We call it schedule reboot, since fw_upgrade module must schedule a reboot after n seconds when done */
enum dfu_status_flag {
	DFU_STATUS_IDLE = 0,
	DFU_STATUS_IN_PROGRESS = 1,
	DFU_STATUS_SUCCESS_REBOOT_SCHEDULED = 2
};

struct dfu_status_event {
	struct event_header header;

	/* Might be beneficial to know what triggered the DFU process */
	enum dfu_trigger_type trigger_type;
	/* Status of the firmware upgrade */
	enum dfu_status_flag dfu_status;
};

EVENT_TYPE_DECLARE(dfu_fragment_event);
EVENT_TYPE_DECLARE(dfu_status_event);

#endif /* _FW_UPGRADE_EVENTS_H_ */