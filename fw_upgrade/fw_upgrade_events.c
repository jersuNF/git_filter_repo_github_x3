/*
 * Copyright (c) 2021 Nofence AS
 */

#include "fw_upgrade_events.h"
#include <logging/log.h>
#include <stdio.h>

#define LOG_MODULE_NAME fw_upgrade_events
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_FW_UPGRADE_LOG_LEVEL);

/**
 * @brief Fragment recieved event function for debugging/information. 
 *        Uses the log to make it easier to
 *        debug what is happening on the event bus.
 * 
 * @param[in] ev event_header for given event.
 * @param[in] buf triggered event's log event buffer.
 * @param[in] buf_len length of the buffer received.
 */
static int log_dfu_fragment_event(const struct event_header *eh, char *buf,
				  size_t buf_len)
{
	struct dfu_fragment_event *event = cast_dfu_fragment_event(eh);
	return snprintf(buf, buf_len, "File_size=%d", event->file_size);
}

EVENT_TYPE_DEFINE(dfu_fragment_event, true, log_dfu_fragment_event, NULL);

/**
 * @brief DFU status event function for debugging/information. 
 *        Uses the log to make it easier to
 *        debug what is happening on the event bus.
 * 
 * @param[in] ev event_header for given event.
 * @param[in] buf triggered event's log event buffer.
 * @param[in] buf_len length of the buffer received.
 */
static int log_dfu_status_event(const struct event_header *eh, char *buf,
				size_t buf_len)
{
	struct dfu_status_event *event = cast_dfu_status_event(eh);
	return snprintf(buf, buf_len, "DFU_status=%d", event->dfu_status);
}

EVENT_TYPE_DEFINE(dfu_status_event, true, log_dfu_status_event, NULL);