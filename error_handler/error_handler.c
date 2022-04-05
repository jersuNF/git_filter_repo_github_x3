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

#include <date_time.h>

#include "error_handler.h"

K_MSGQ_DEFINE(err_container_msgq, sizeof(struct error_container), 4, 4);

LOG_MODULE_REGISTER(error_handler, CONFIG_ERROR_HANDLER_LOG_LEVEL);

/**
 * @brief Processing function for fatal errors. 
 *        See error_event.h for param desc.
 */
static inline void process_fatal(enum error_sender_module sender, int code)
{
	LOG_DBG("Received a fatal error (%d) from sender %d ", code, sender);

	/* Reboot on fatal events */
	struct pwr_reboot_event *r_ev = new_pwr_reboot_event();
	EVENT_SUBMIT(r_ev);
}

/**
 * @brief Processing function for normal errors. 
 *        See error_event.h for param desc. 
 */
static inline void process_error(enum error_sender_module sender, int code)
{
	LOG_DBG("Received a normal error (%d) from sender %d ", code, sender);
}

/**
 * @brief Processing function for warnings. See error_event.h for param desc. 
 */
static inline void process_warning(enum error_sender_module sender, int code)
{
	LOG_DBG("Received a warning (%d) from sender %d ", code, sender);
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
	if (is_error_event(eh)) {
		struct error_event *ev = cast_error_event(eh);

		struct error_container err_c = { .sender = ev->sender,
						 .code = ev->code,
						 .severity = ev->severity };

		if (ev->dyndata.size <= CONFIG_ERROR_USER_MESSAGE_SIZE) {
			memcpy(&err_c.msg, &ev->dyndata.data, ev->dyndata.size);
		} else {
			char *no_err_msg = "No error msg or corrupt msg.";
			memcpy(err_c.msg, no_err_msg, strlen(no_err_msg));
		}

		while (k_msgq_put(&err_container_msgq, &err_c, K_NO_WAIT) !=
		       0) {
			/* Message queue is full: purge old data & try again */
			k_msgq_purge(&err_container_msgq);
		}
		return false;
	}
	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(error_handler, event_handler);
EVENT_SUBSCRIBE(error_handler, error_event);

void error_handler_thread_fn()
{
	while (true) {
		struct error_container err_container;
		int err = k_msgq_get(&err_container_msgq, &err_container,
				     K_FOREVER);
		if (err) {
			LOG_ERR("Error: Retrieving err_message queue %i", err);
			continue;
		}

		switch (err_container.severity) {
		case ERR_SEVERITY_FATAL:
			process_fatal(err_container.sender, err_container.code);
			break;
		case ERR_SEVERITY_ERROR:
			process_error(err_container.sender, err_container.code);
			break;
		case ERR_SEVERITY_WARNING:
			process_warning(err_container.sender,
					err_container.code);
			break;
		default:
			LOG_ERR("Unknown error severity.");
			continue;
		}

		/** @todo What should we do about the error message? Also check if
		 *        the string is null terminated to prevent undefined behaviour?
		 */
		LOG_DBG("%s", log_strdup(err_container.msg));

		/** @todo Notify server / bluetooth about error? */

		/* Store part of the error to system diagnostic partition. */
		int64_t unix_time;
		if (date_time_now(&unix_time)) {
			/* If neither GNSS or modem has updated 
			 * the timestamp, set it to 0.
	 	 	 */
			unix_time = 0;
		}

		system_diagnostic_t sys_diag = { .error_code =
							 err_container.code,
						 .sender = err_container.sender,
						 .uptime = k_uptime_get(),
						 .unix_time = unix_time };

		size_t sys_diag_len = sizeof(system_diagnostic_t);
		err = stg_write_system_diagnostic_log((uint8_t *)&sys_diag,
						      sys_diag_len);
		if (err) {
			LOG_ERR("Cannot write system diagnostic to external flash %i",
				err);
		}
	}
}

K_THREAD_DEFINE(error_thread, CONFIG_ERROR_THREAD_STACK_SIZE,
		error_handler_thread_fn, NULL, NULL, NULL,
		CONFIG_ERROR_THREAD_PRIORITY, 0, 0);