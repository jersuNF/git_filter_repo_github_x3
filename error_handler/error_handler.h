/*
 * Copyright (c) 2021 Nofence AS
 */

#ifndef _ERROR_HANDLER_H_
#define _ERROR_HANDLER_H_

#include <zephyr.h>
#include "event_manager.h"
#include "error_event.h"

struct error_container {
	enum error_sender_module sender;
	int code;
	enum error_severity severity;
	uint8_t msg[CONFIG_ERROR_USER_MESSAGE_SIZE];
};

#endif /* _ERROR_HANDLER_H_ */