/*
 * Copyright (c) 2021 Nofence AS
 */

#include "fw_upgrade_events.h"

/**
 * @brief Fragment recieved event function for debugging/information. 
 * 		  Uses the profiler tool to make it easier to
 * 		  debug what is happening on the event bus.
 * 
 * @param[in] buf triggered event's log event buffer.
 * @param[in] ev event_header for given event.
 */
static void profile_dfu_fragment_event(struct log_event_buf *buf,
				       const struct event_header *eh)
{
}

EVENT_INFO_DEFINE(dfu_fragment_event, ENCODE(), ENCODE(),
		  profile_dfu_fragment_event);

EVENT_TYPE_DEFINE(dfu_fragment_event, true, NULL, &dfu_fragment_event_info);

/**
 * @brief Status event profiler function for debugging/information. 
 * 		  Uses the profiler tool to make it easier to
 * 		  debug what is happening on the event bus.
 * 
 * @param[in] buf triggered event's log event buffer.
 * @param[in] ev event_header for given event.
 */
static void profile_dfu_status_event(struct log_event_buf *buf,
				     const struct event_header *eh)
{
}

EVENT_INFO_DEFINE(dfu_status_event, ENCODE(), ENCODE(),
		  profile_dfu_status_event);

EVENT_TYPE_DEFINE(dfu_status_event, true, NULL, &dfu_status_event_info);