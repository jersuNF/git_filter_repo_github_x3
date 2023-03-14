/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _CMD_STORAGE_H_
#define _CMD_STORAGE_H_

#include "diagnostics_types.h"
#include "commander_def.h"

int commander_storage_handler(enum diagnostics_interface interface, uint8_t cmd, uint8_t *data,
			      uint32_t size);

#endif /* _CMD_STORAGE_H_ */