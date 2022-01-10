/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include "error_event.h"
#include <logging/log.h>

#define LOG_MODULE_NAME error_handler
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_ERROR_HANDLER_LOG_LEVEL);

/**
 * @brief Processing function for fatal errors. 
 * 
 * @param[in] sender which module triggered the warning.
 * @param[in] code error code from the sender.
 * @param[in] msg custom user message attached to the message. Can be NULL.
 */
static inline void process_fatal(enum error_sender_module sender, int code,
				 char *msg)
{
	/* Process fatal errors here. */
}

/**
 * @brief Processing function for normal errors. 
 * 
 * @param[in] sender which module triggered the warning.
 * @param[in] code error code from the sender.
 * @param[in] msg custom user message attached to the message. Can be NULL.
 */
static inline void process_error(enum error_sender_module sender, int code,
				 char *msg)
{
	/* Process normal errors here. */
}

/**
 * @brief Processing function for warnings. 
 * 
 * @param[in] sender which module triggered the warning.
 * @param[in] code error code from the sender.
 * @param[in] msg custom user message attached to the message. Can be NULL.
 */
static inline void process_warning(enum error_sender_module sender, int code,
				   char *msg)
{
	/* Process warnings here. */
}

/**
 * @brief Main event handler function. 
 * 
 * @param[in] eh Event_header for the if-chain to 
 *               use to recognize which event triggered.
 * 
 * @return True or false based on if we want to consume the event or not.
 */
static bool event_handler(const struct event_header *eh)
{
	//int err;
	if (is_error_event(eh)) {
		struct error_event *ev = cast_error_event(eh);
		switch (ev->severity) {
		case ERR_SEVERITY_FATAL:
			process_fatal(ev->sender, ev->code, ev->user_message);
			break;
		case ERR_SEVERITY_ERROR:
			process_error(ev->sender, ev->code, ev->user_message);
			break;
		case ERR_SEVERITY_WARNING:
			process_warning(ev->sender, ev->code, ev->user_message);
			break;
		default:
			LOG_ERR("Unknown error severity.");
			break;
		}
		return false;
	}
	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, error_event);