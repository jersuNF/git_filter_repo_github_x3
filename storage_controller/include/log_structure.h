/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _LOG_DEF_H_
#define _LOG_DEF_H_

#include <zephyr.h>

#define LOG_ENTRY_BUF_SIZE 256

/**
 * This structure defines all messages stored in the external flash.
 *
 * @param header Header of entry in flash
 * @param buf Actual data, size is header.len bytes
 *
 */
typedef struct {
	struct {
		/** Length of body. */
		uint16_t len;
	} header;
	uint8_t buf[LOG_ENTRY_BUF_SIZE];
} log_rec_t;

#endif /* _LOG_DEF_H_ */