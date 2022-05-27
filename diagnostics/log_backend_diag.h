/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _LOG_BACKEND_DIAG_H_
#define _LOG_BACKEND_DIAG_H_

#include "diagnostics_types.h"

#include <device.h>

/** @brief Struct holding pointers to sending and passthrough enable actions.
 */
struct log_backend_diag_action {
	void (*send_resp)(enum diagnostics_interface, const uint8_t*, uint32_t);
	int (*thru_enable)(enum diagnostics_interface, const struct device*);
};

/**
 * @brief Used to initialize log backend for diagnostics.
 * 
 * @param[in] actions Struct holding function pointers to required actions in backend. 
 * 
 * @return 0 on success, otherwise negative errno.
 */
int log_backend_diag_init(struct log_backend_diag_action* actions);

/**
 * @brief Enables log backen for diagnostics. 
 * 
 * @param[in] intf Diagnostics interface for logging.
 * 
 * @return 0 on success, otherwise negative errno.
 */
int log_backend_diag_enable(enum diagnostics_interface intf);

/**
 * @brief Disables log backend. 
 * 
 * @return 0 on success, otherwise negative errno.
 */
int log_backend_diag_disable(void);

#endif /* _LOG_BACKEND_DIAG_H_ */
