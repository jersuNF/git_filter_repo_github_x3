#include <zephyr.h>
#include "fw_upgrade_events.h"

static inline void apply_fragment(uint8_t *fragment, size_t fragment_size,
			   size_t file_size,
			   enum dfu_trigger_type fragment_origin)
{

}

/**
 * @brief Main event handler function that handles all the events this module subscribes to,
 * basically a large *switch* case using if's and prefedined event triggers to check against given
 * event_header param.
 * @param eh event_header for the if-chain to use to recognize which event triggered
 */
static bool event_handler(const struct event_header *eh)
{
	if (is_dfu_fragment_event(eh)) {
		struct dfu_fragment_event *event = cast_dfu_fragment_event(eh);

        /* Call function that writes given fragment to internal flash S1 */
		apply_fragment((uint8_t*)&event->fragment, event->fragment_size,
			       event->file_size, event->trigger_type);

		/* Consume event and wait for next update*/
		return true;
	}
	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, dfu_fragment_event);