/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _COMMANDER_DEF_H_
#define _COMMANDER_DEF_H_

typedef struct commander_header_t {
	uint8_t group;
	uint8_t command;
	uint16_t checksum;
};

#endif /* _COMMANDER_DEF_H_ */