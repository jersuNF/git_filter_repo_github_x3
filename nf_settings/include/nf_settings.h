
/*
* Copyright (c) 2022 Nofence AS
*/

#ifndef X3_FW_NF_SETTINGS_H
#define X3_FW_NF_SETTINGS_H

#include <device.h>

/**
 * @brief max size of the IPV4 host and port string received from the
 * server.
 * @example: 193.146.222.555:123456 = 22 characters
 */
#define EEP_HOST_PORT_BUF_SIZE 24

/**
 * @brief initialises the eeprom API
 * @param[in] device pointer to the EEPROM device to use for subsequent read/writes
 */
void eep_init(const struct device *dev);

/**
 * @brief writes the persistent collar serial number.
 * @param[in] serial the serial number to persist in non-volatile storage
 *
 * @return 0 on success, otherwise negative error code.
 */
int eep_write_serial(uint32_t serial);

/**
 * @brief reads the persistent collar serial number. Might be cached internally
 * @param[out] p_serial pointer to the serial number to receive the value
 *
 * @return 0 on success, otherwise negative error code.
 */
int eep_read_serial(uint32_t *p_serial);

/**
 * @brief writes the server host-port string to persisted storage
 * @param[in] host_port pointer to a null-terminated host-port string e.g.
 * @p 192.177.255.345:987654. The length of the string must not exceed
 * @p EEP_HOST_PORT_BUF_SIZE -1
 * @return 0 on success, otherwise negative errror code
 */
int eep_write_host_port(const char *host_port);

/**
 * @brief reads the server host-port string from persisted storage
 * @param[out] host_port pointer to buffer where the string is stored
 * @param bufsize size of the buffer
 * @return 0 on success, otherwise negative errror code
 */
int eep_read_host_port(char *host_port, size_t bufsize);

#endif /* X3_FW_NF_SETTINGS_H */
