/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _FLASH_MEMORY_H_
#define _FLASH_MEMORY_H_

#include <zephyr.h>

#define MEM_REC_ENTRY_BUF_SIZE 256
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

		/** USED on AnoData as AnoID, otherwise UNUSED. */
		uint16_t tag; //

		/** Used on AnoData as AnoStartID otherwise UNUSED. */
		uint32_t ID;
	} header;
	uint8_t buf[MEM_REC_ENTRY_BUF_SIZE];
} mem_rec;

#endif /* _FLASH_MEMORY_H_ */