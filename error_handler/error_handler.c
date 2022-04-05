/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include "error_event.h"
#include "pwr_event.h"
#include "system_diagnostic_structure.h"
#include "storage.h"

#include <logging/log.h>
#include <power/reboot.h>

#define LOG_MODULE_NAME error_handler
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_ERROR_HANDLER_LOG_LEVEL);

/**
 * @brief Processing function for fatal errors. 
 *        See error_event.h for param desc.
 */
static inline void process_fatal(enum error_sender_module sender, int code,
				 char *msg, size_t msg_len)
{
	LOG_INF("Received a fatal error (%d) from sender %d ", code, sender);

	/* Log error in external flash */
	int64_t uptime = k_uptime_get();
	int64_t unix_time = 0; /* TODO: ADD Eivind's function */
	system_diagnostic_t sys_diag = { .error_code = code,
					 .sender = sender,
					 .uptime = uptime,
					 .unix_time = unix_time };

	size_t sys_diag_len = sizeof(system_diagnostic_t);
	stg_write_system_diagnostic_log((uint8_t *)&sys_diag, sys_diag_len);

	/* Reboot on fatal events */
	struct pwr_reboot_event *r_ev = new_pwr_reboot_event();
	EVENT_SUBMIT(r_ev);
}

/**
 * @brief Processing function for normal errors. 
 *        See error_event.h for param desc. 
 */
static inline void process_error(enum error_sender_module sender, int code,
				 char *msg, size_t msg_len)
{
	LOG_INF("Received a normal error (%d) from sender %d ", code, sender);

	/* Log error in external flash */
	int64_t uptime = k_uptime_get();
	int64_t unix_time = 0; /* TODO: ADD Eivind's function */
	system_diagnostic_t sys_diag = { .error_code = code,
					 .sender = sender,
					 .uptime = uptime,
					 .unix_time = unix_time };

	size_t sys_diag_len = sizeof(system_diagnostic_t);
	stg_write_system_diagnostic_log((uint8_t *)&sys_diag, sys_diag_len);
}

/**
 * @brief Processing function for warnings. See error_event.h for param desc. 
 */
static inline void process_warning(enum error_sender_module sender, int code,
				   char *msg, size_t msg_len)
{
	LOG_INF("Received a warning (%d) from sender %d ", code, sender);

	/* Log warning in external flash */
	int64_t uptime = k_uptime_get();
	int64_t unix_time = 0; /* TODO: ADD Eivind's function */
	system_diagnostic_t sys_diag = { .error_code = code,
					 .sender = sender,
					 .uptime = uptime,
					 .unix_time = unix_time };

	size_t sys_diag_len = sizeof(system_diagnostic_t);
	stg_write_system_diagnostic_log((uint8_t *)&sys_diag, sys_diag_len);
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
			process_fatal(ev->sender, ev->code, ev->dyndata.data,
				      ev->dyndata.size);
			break;
		case ERR_SEVERITY_ERROR:
			process_error(ev->sender, ev->code, ev->dyndata.data,
				      ev->dyndata.size);
			break;
		case ERR_SEVERITY_WARNING:
			process_warning(ev->sender, ev->code, ev->dyndata.data,
					ev->dyndata.size);
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