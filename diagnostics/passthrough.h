/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _PASSTHROUGH_H_
#define _PASSTHROUGH_H_

#include "diagnostics_types.h"

#include <zephyr.h>
#include <device.h>

int passthrough_init(void);
int passthrough_enable(enum diagnostics_interface intf, 
			const struct device *dev);
int passthrough_disable(void);
void passthrough_get_enabled_interface(enum diagnostics_interface *intf);
uint32_t passthrough_write_data(uint8_t* data, uint32_t size);
uint32_t passthrough_claim_read_data(uint8_t** data);
int passthrough_finish_read_data(uint32_t size);

#endif /* _PASSTHROUGH_H_ */