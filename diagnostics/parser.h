/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _PARSER_H_
#define _PARSER_H_

#include "diagnostics_types.h"

#include <device.h>

struct parser_action {
	void (*send_resp)(enum diagnostics_interface, const uint8_t*, uint32_t);
	int (*thru_enable)(enum diagnostics_interface, const struct device*);
};

int parser_init(struct parser_action* actions);
uint32_t parser_handle(enum diagnostics_interface interface, uint8_t* data, uint32_t size);

#endif /* _PARSER_H_ */