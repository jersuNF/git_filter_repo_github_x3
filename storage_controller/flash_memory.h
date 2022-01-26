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

/**
 * This structure defines flash sectors
 * FYI - https://youtrack.axbit.com/youtrack/issue/NOF-311
 */
typedef struct {
	/** Pointer to start of the buffer in the FLASH. */
	uint32_t start_ptr;

	/** Length of the sector. */
	uint32_t length;

	void (*eepromWriteStart)(uint8_t addr[4]);
	void (*eepromWriteEnd)(uint8_t addr[4]);
	void (*eepromWriteTransferred)(uint8_t addr[4]);

	void (*eepromReadStart)(uint8_t addr[4]);
	void (*eepromReadEnd)(uint8_t addr[4]);
	void (*eepromReadTransferred)(uint8_t addr[4]);
} flash_region;

/**
 * This structure defines flash address
 * Bytes 1-3: Page address
 * byte 4: address of cell in page
 */
typedef struct {
	uint8_t x[4];
} flash_address;

#define MEM_EQUAL_ADDR(ad1, ad2)                                               \
	((ad1)[0] == (ad2)[0] && (ad1)[1] == (ad2)[1] && (ad1)[2] == (ad2)[2])
#define MEM_ADDR4_TO_UINT32(addr)                                              \
	(((uint32_t)(addr)[2] << 16) | ((uint32_t)(addr)[1] << 8) |            \
	 (uint32_t)(addr)[0])
#define MEM_FLASH_TO_UINT32(addr) MEM_ADDR4_TO_UINT32((addr).x)
#define MEM_FLASH_ADDRESS_AS_ID(addr) (*(uint32_t *)&(addr))
#define MEM_ID_AS_FLASH_ADDRESS(id) (*(FLASH_ADDRESS *)&(id))
#define MEM_IS_EMPTY_FLASH_ADDRESS(addr)                                       \
	((addr).x[0] == 0xFF && (addr).x[1] == 0xFF && (addr).x[2] == 0xFF)
#define MEM_EMPTY_FLASH_ADDRESS                                                \
	{                                                                      \
		0xFF, 0xFF, 0xFF, 0xFF                                         \
	}

#define MEM_SET_EMPTY_ADDRESS(addr)                                            \
	do {                                                                   \
		(addr).x[0] = 0xFF;                                            \
		(addr).x[1] = 0xFF;                                            \
		(addr).x[2] = 0xFF;                                            \
		(addr).x[3] = 0xFF;                                            \
	} while (0)
#define MEM_FLASHPAGE(addr) (MEM_FLASH_TO_UINT32((addr)) / FLASH_PAGESIZE)
#define MEM_ADDR_4K_BLOCK_INDEX(addr) (MEM_FLASH_TO_UINT32((addr)) / 4096)
#define MEM_ADDR_4K_BLOCK_INDEX_LEN(addr, len)                                 \
	((MEM_FLASH_TO_UINT32((addr)) + len) / 4096)
#define MEM_4K_BLOCK_INDEX(x) ((x) / 4096)

#define MEM_SUCCESS 1
#define MEM_EMPTY_RECORD 2
#define MEM_EOF 3
#define MEM_ERROR_RECORD_TOO_LARGE 6
#define MEM_ERROR_INVALID_ADDRESS 7
#define MEM_ERROR_NOT_VALID_ID_UNUSED_SINCE_31 8
#define MEM_ERROR_NOT_FOUND_UNUSED_SINCE_31 9
#define MEM_ERROR_FLASH_ERASE 10
#define MEM_ERROR_FLASH_WRITE 11
#define MEM_ERROR_FLASH_READ 12
#define MEM_ERROR_UNIX_DATE 13
#define MEM_ERROR_UNIX_ 13

/** 512k size. */
#define MEM_MAX_SIZE_LOG 0x80000

/** 128k size. */
#define MEM_MAX_SIZE_ANO 0x20000

/** 256k size. */
#define MEM_MAX_SIZE_FIRMWARE 0x40000

/** 32k size. */
#define MEM_MAX_SIZE_FENCE 0x8000

#define MEM_START_LOG 0
#define MEM_START_ANO (MEM_START_LOG + MEM_MAX_SIZE_LOG)
#define MEM_START_FIRMWARE (MEM_START_ANO + MEM_MAX_SIZE_ANO)
#define MEM_START_FENCE (MEM_START_FIRMWARE + MEM_MAX_SIZE_FIRMWARE)

#define MEM_ID_EOF_MARKER ((uint32_t)0xAABBCCDD)

#endif /* _FLASH_MEMORY_H_ */