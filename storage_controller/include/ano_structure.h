/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _ANO_DEF_H_
#define _ANO_DEF_H_

#include <zephyr.h>

#define ANO_ENTRY_BUF_SIZE 256

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

		/** USED as AnoID. */
		uint16_t tag; //

		/** Used as AnoStartID. */
		uint32_t ID;

	} header;
	uint8_t buf[ANO_ENTRY_BUF_SIZE];
} ano_rec;

#endif /* _ANO_DEF_H_ */