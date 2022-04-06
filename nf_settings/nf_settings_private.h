
/*
* Copyright (c) 2022 Nofence AS
*/

#ifndef X3_FW_NF_EEPROM_PRIVATE_H
#define X3_FW_NF_EEPROM_PRIVATE_H

#include <stdint.h>
#include <nf_settings.h>

/**
 * @brief Convenience structure to define the Zephyr eeprom address space
 * for storing/retrieving values
 *
 * Typically, we use the C library macro @p offsetof to determine
 * the address to use with Zephyr @p eeprom_read and @p eeprom_write
 *
 * @note You are not meant to instantiate this structure outside the Nofence
 * eeprom API.
 */
__packed struct eemem {
	/** The collar serial number, must always be the first member */
	uint32_t uid;
	/** Configurable 0-terminated host:port, e.g. @p 192.176.777.888:987654\0 */
	char host_port[EEP_HOST_PORT_BUF_SIZE];
};

#endif //X3_FW_NF_EEPROM_PRIVATE_H
