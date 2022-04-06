/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _COMMANDER_DEF_H_
#define _COMMANDER_DEF_H_

typedef struct __attribute__((packed)) {
	/* Overall group context of the command */
	uint8_t group;
	/* Command to execute, specific for each group context */
	uint8_t command;
	/* CRC16 CCITT */
	uint16_t checksum;
} commander_header_t;

typedef struct __attribute__((packed)) {
	/* Overall group context of the command */
	uint8_t group;
	/* Command to execute, specific for each group context */
	uint8_t command;
	/* Response type */
	uint8_t response;
	/* CRC16 CCITT */
	uint16_t checksum;
} commander_resp_t;

typedef enum {
	ACK = 0x00,
	DATA = 0x01,

	CHK_FAILED = 0xC0,
	NOT_ENOUGH = 0xD0,
	ERROR = 0xE0,
	UNKNOWN_CMD = 0xFC,
	UNKNOWN_GRP = 0xFE,
	UNKNOWN = 0xFF
} commander_resp_t;

typedef enum {
	SYSTEM = 0x00,
	SETTINGS = 0x01,
	STIMULATOR = 0x02,
	STORAGE = 0x03,
} commander_group_t;

typedef enum {
	READ = 0x00,
	WRITE = 0x01,
	ERASE_ALL = 0xEA
} settings_cmd_t;

typedef enum {
	GNSS_DATA = 0x10,
	BUZZER_WARN = 0xB0,
	EP_RELEASE = 0xE0,
} stimulator_cmd_t;

#endif /* _COMMANDER_DEF_H_ */