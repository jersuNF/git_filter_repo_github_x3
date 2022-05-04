
/*
* Copyright (c) 2022 Nofence AS
*/

#ifndef X3_FW_NF_SETTINGS_H
#define X3_FW_NF_SETTINGS_H

#include <device.h>

/** 
 * @brief Convenience enums for easily calling read/write functions
 *        where we simply can call eep_uint8_write with an enum and a value
 *        to calculate offsetof.
 */
typedef enum { EEP_UID = 0, EEP_WARN_CNT_TOT } eep_uint32_enum_t;

typedef enum {
	EEP_ACC_SIGMA_NOACTIVITY_LIMIT = 0,
	EEP_OFF_ANIMAL_TIME_LIMIT_SEC,
	EEP_ACC_SIGMA_SLEEP_LIMIT,
	EEP_ZAP_CNT_TOT,
	EEP_ZAP_CNT_DAY
} eep_uint16_enum_t;

typedef enum {
	EEP_WARN_MAX_DURATION = 0,
	EEP_WARN_MIN_DURATION,
	EEP_PAIN_CNT_DEF_ESCAPED,
	EEP_COLLAR_MODE,
	EEP_FENCE_STATUS,
	EEP_COLLAR_STATUS,
	EEP_TEACH_MODE_FINISHED
} eep_uint8_enum_t;

/**
 * @brief max size of the IPV4 host and port string received from the
 * server.
 * @example: 193.146.222.555:123456 = 22 characters
 */
#define EEP_HOST_PORT_BUF_SIZE 24

#define EEP_BLE_SEC_KEY_LEN 8
/**
 * @brief initialises the eeprom API
 * @param[in] device pointer to the EEPROM device to use for subsequent read/writes
 */
void eep_init(const struct device *dev);

/**
 * @brief writes the server host-port string to persisted storage
 * @param[in] host_port pointer to a null-terminated host-port string e.g.
 * @p 192.177.255.345:987654. The length of the string must not exceed
 * @p EEP_HOST_PORT_BUF_SIZE -1
 * @return 0 on success, otherwise negative error code
 */
int eep_write_host_port(const char *host_port);

/**
 * @brief reads the server host-port string from persisted storage
 * @param[out] host_port pointer to buffer where the string is stored
 * @param bufsize size of the buffer
 * @return 0 on success, otherwise negative error code
 */
int eep_read_host_port(char *host_port, size_t bufsize);

/**
 * @brief writes the ble_security key to persisted storage
 * @param[in] ble_sec_key pointer to security key array.
 * @param[in] bufsize length of data to be written
 *
 * @return 0 on success, otherwise negative errror code
 */
int eep_write_ble_sec_key(uint8_t *ble_sec_key, size_t bufsize);

/**
 * @brief reads the ble_security key from persisted storage
 * @param[in] ble_sec_key pointer to security key array.
 * @param[in] bufsize length of data to be written
 *
 * @return 0 on success, otherwise negative errror code
 */
int eep_read_ble_sec_key(uint8_t *ble_sec_key, size_t bufsize);

int eep_uint8_read(eep_uint8_enum_t field, uint8_t *value);
int eep_uint8_write(eep_uint8_enum_t field, uint8_t value);

int eep_uint16_read(eep_uint16_enum_t field, uint16_t *value);
int eep_uint16_write(eep_uint16_enum_t field, uint16_t value);

int eep_uint32_read(eep_uint32_enum_t field, uint32_t *value);
int eep_uint32_write(eep_uint32_enum_t field, uint32_t value);

#endif /* X3_FW_NF_SETTINGS_H */