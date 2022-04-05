/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _SYSTEM_DIAGNOSTIC_LOG_H
#define _SYSTEM_DIAGNOSTIC_LOG_H

#include <zephyr.h>
#include <date_time.h>
#include "error_event.h"

/** @todo Fix this struct to contain what is needed. */
typedef struct __attribute__((packed)) {
	int32_t error_code;
	int64_t uptime;
	int64_t unix_time;
	enum error_sender_module sender;
} system_diagnostic_t;

#endif /* _SYSTEM_DIAGNOSTIC_LOG_H */