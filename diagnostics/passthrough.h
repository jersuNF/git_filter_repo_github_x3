/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _PASSTHROUGH_H_
#define _PASSTHROUGH_H_

#include "diagnostics_types.h"

#include <zephyr.h>
#include <device.h>

/**
 * @brief Used to initialize the passthrough. 
 * 
 * @return 0 on success, otherwise negative errno.
 */
int passthrough_init(void);

/**
 * @brief Enables passthrough between diagnostics interface and UART device. 
 * 
 * @param[in] intf Diagnostics interface for passthrough. 
 * @param[in] dev UART device for passthrough. 
 * 
 * @return 0 on success, otherwise negative errno.
 */
int passthrough_enable(enum diagnostics_interface intf, const struct device *dev);

/**
 * @brief Disables passthrough. 
 * 
 * @return 0 on success, otherwise negative errno.
 */
int passthrough_disable(void);

/**
 * @brief Gets current enabled diagnostics interface. 
 * 
 * @param[out] intf Diagnostics interface for passthrough. 
 *                  Gets set to DIAGNOSTICS_NONE when disabled. 
 */
void passthrough_get_enabled_interface(enum diagnostics_interface *intf);

/**
 * @brief Writes data to passthrough device. 
 * 
 * @param[in] data Buffer of data to send. 
 * @param[in] size Size of data to send. 
 * 
 * @return Number of bytes written. 
 */
uint32_t passthrough_write_data(uint8_t *data, uint32_t size);

/**
 * @brief Claims data received from passthrough device. Data claimed 
 *        must be finished by calling passthrough_finish_read_data. 
 * 
 * @param[out] data Buffer holding received data. 
 * 
 * @return Number of bytes available. 
 */
uint32_t passthrough_claim_read_data(uint8_t **data);

/**
 * @brief Finishes data read and deletes specified size.
 * 
 * @param[in] size Number of bytes that should be deleted. 
 * 
 * @return 0 on success, otherwise negative errno.
 */
int passthrough_finish_read_data(uint32_t size);

#endif /* _PASSTHROUGH_H_ */