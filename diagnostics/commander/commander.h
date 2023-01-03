/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _COMMANDER_H_
#define _COMMANDER_H_

#include "diagnostics_types.h"

#include <device.h>

/** @brief Struct holding pointers to sending and passthrough enable actions.
 */
struct commander_action {
	void (*send_resp)(enum diagnostics_interface, const uint8_t *, uint32_t);
	int (*thru_enable)(enum diagnostics_interface, const struct device *);
};

/**
 * @brief Used to initialize the commander.
 * 
 * @param[in] actions Struct holding function pointers to required actions in commander. 
 * 
 * @return 0 on success, otherwise negative errno.
 */
int commander_init(struct commander_action *actions);

/**
 * @brief Commander handle function will parse provided data and call requested actions. 
 * 
 * @param[in] interface Interface from where the data was received. 
 * @param[in] data Buffer of data. 
 * @param[in] size Size of data. 
 * 
 * @return Number of bytes parsed in data buffer. 
 */
uint32_t commander_handle(enum diagnostics_interface interface, uint8_t *data, uint32_t size);

#endif /* _COMMANDER_H_ */