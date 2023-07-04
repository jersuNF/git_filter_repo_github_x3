/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef STG_CONFIG_H
#define STG_CONFIG_H

#include <zephyr.h>

/* Length of a null-terminated IPv4 host/port string */
#define STG_CONFIG_HOST_PORT_BUF_LEN 24
/* Length of a Bluetooth Security Key */
#define STG_CONFIG_BLE_SEC_KEY_LEN 8
/*
 * Size of the URAT argument buffer, including the C null-terminator. Note that
 * This field is fixed, even if the protobuf definition should change
 * its size for a reason or another.
 */
#define STG_CONFIG_URAT_ARG_BUF_SIZE 10

/**
 * @brief Identifiers for configuration parameters.
 * @note The enum members are explictly given values in order to
 * stress that a value should not be changed without migration, and
 * to ease merging of branches.
 */
typedef enum {
	/* NB! Do NOT change or reorder the identifiers of NVS Id-data pairs without 
	 * a MIGRATION of existing parameters for all firmware versions  */
	STG_U8_WARN_MAX_DURATION = 0,
	STG_U8_WARN_MIN_DURATION = 1,
	STG_U8_PAIN_CNT_DEF_ESCAPED = 2,
	STG_U8_COLLAR_MODE = 3,
	STG_U8_FENCE_STATUS = 4,
	STG_U8_COLLAR_STATUS = 5,
	STG_U8_TEACH_MODE_FINISHED = 6,
	STG_U8_EMS_PROVIDER = 7,
	STG_U8_PRODUCT_RECORD_REV = 8,
	STG_U8_BOM_MEC_REV = 9,
	STG_U8_BOM_PCB_REV = 10,
	STG_U8_HW_VERSION = 11,
	STG_U8_KEEP_MODE = 12,
	STG_U8_RESET_REASON = 13,
	STG_U16_ACC_SIGMA_NOACTIVITY_LIMIT = 14,
	STG_U16_OFF_ANIMAL_TIME_LIMIT_SEC = 15,
	STG_U16_ACC_SIGMA_SLEEP_LIMIT = 16,
	STG_U16_ZAP_CNT_TOT = 17,
	STG_U16_ZAP_CNT_DAY = 18,
	STG_U16_PRODUCT_TYPE = 19,
	STG_U32_UID = 20,
	STG_U32_WARN_CNT_TOT = 21,
	STG_STR_HOST_PORT = 22,
	STG_BLOB_BLE_KEY = 23,
	STG_U8_MODEM_INSTALLING = 24,
	/* There's been some mismatch here with different branches.
	 * Let's reserve these params to be sure to not have any issue
	 */
	STG_PARAM_RESERVED1 = 25,
	STG_PARAM_RESERVED2 = 26,
	STG_PARAM_RESERVED3 = 27,
	STG_PARAM_RESERVED4 = 28,
	STG_PARAM_RESERVED5 = 29,
	STG_PARAM_RESERVED6 = 30,
	STG_PARAM_RESERVED7 = 31,
	STG_U32_DIAGNOSTIC_FLAGS_TODO_UNUSED = 32,
	STG_STR_MODEM_URAT_ARG = 33,
	STG_U32_ANO_TIMESTAMP = 34,
	STG_U16_ANO_START_ID = 35,
	STG_U16_ANO_ID = 36,
	STG_U16_LAST_GOOD_ANO_ID = 37,
	STG_U32_MIGRATED_VERSION = 38
} stg_config_param_id_t;

#ifdef CONFIG_TEST
void stg_config_reset_for_unit_tests();

struct nvs_fs *stg_config_get_file_system();
#endif

/**
 * @brief Initialization of the flash configuration storage. 
 * @return 0 if successful, otherwise a negative error code.
 */
int stg_config_init(void);

/**
 * @brief Reads the u8 config parameter as specified by the identifier.
 * @param id The identifier of the config parameter (See stg_config_param_id_t).
 * @param value Pointer to the u8 data value.
 * @return 0 if successful, otherwise a negative error code. 
 */
int stg_config_u8_read(stg_config_param_id_t id, uint8_t *value);

/**
 * @brief Writes to the u8 config parameter as specified by the identifier.
 * @param id The identifier of the config parameter (See stg_config_param_id_t).
 * @param value The u8 data value to write.
 * @return 0 if successful, otherwise a negative error code. 
 */
int stg_config_u8_write(stg_config_param_id_t id, const uint8_t value);

/**
 * @brief Reads the u16 config parameter as specified by the identifier.
 * @param id The identifier of the config parameter (See stg_config_param_id_t).
 * @param value Pointer to the u16 data value.
 * @return 0 if successful, otherwise a negative error code. 
 */
int stg_config_u16_read(stg_config_param_id_t id, uint16_t *value);

/**
 * @brief Writes to the u16 config parameter as specified by the identifier.
 * @param id The identifier of the config parameter (See stg_config_param_id_t).
 * @param value The u16 data value to write.
 * @return 0 if successful, otherwise a negative error code. 
 */
int stg_config_u16_write(stg_config_param_id_t id, const uint16_t value);

/**
 * @brief Reads the u32 config parameter as specified by the identifier.
 * @param id The identifier of the config parameter (See stg_config_param_id_t).
 * @param value Pointer to the u32 data value.
 * @return 0 if successful, otherwise a negative error code. 
 */
int stg_config_u32_read(stg_config_param_id_t id, uint32_t *value);

/**
 * @brief Writes to the u32 config parameter as specified by the identifier.
 * @param id The identifier of the config parameter (See stg_config_param_id_t).
 * @param value The u32 data value to write.
 * @return 0 if successful, otherwise a negative error code. 
 */
int stg_config_u32_write(stg_config_param_id_t id, const uint32_t value);

/**
 * @brief Reads the string config parameter as specified by the identifier.
 * @param id The identifier of the config parameter (See stg_config_param_id_t).
 * @param buf Pointer to the data buffer.
 * @param bufsize size of the provided buffer
 * @param len The length of the data on return, not including the terminating '\0'
 * @return 0 if successful, otherwise a negative error code. 
 */
int stg_config_str_read(stg_config_param_id_t id, char *buf, size_t bufsize, uint8_t *len);

/**
 * @brief Writes to the string config parameter as specified by the identifier.
 * @param id The identifier of the config parameter (See stg_config_param_id_t).
 * @param str Pointer to the data buffer.
 * @param len The length of the C-string, not including the terminating 0
 * @return 0 if successful, otherwise a negative error code. 
 */
int stg_config_str_write(stg_config_param_id_t id, const char *str, size_t len);

/**
 * @brief Reads the binary blob config parameter as specified by the identifier.
 * @param id The identifier of the config parameter (See stg_config_param_id_t).
 * @param arr Pointer to the data array.
 * @param len The length of the data.
 * @return 0 if successful, otherwise a negative error code. 
 */
int stg_config_blob_read(stg_config_param_id_t id, uint8_t *arr, uint8_t *len);

/**
 * @brief Writes to binary blob config parameter as specified by the identifier.
 * @param id The identifier of the config parameter (See stg_config_param_id_t).
 * @param arr Pointer to the data array.
 * @param len The length of the data.
 * @return 0 if successful, otherwise a negative error code. 
 */
int stg_config_blob_write(stg_config_param_id_t id, const uint8_t *arr, const uint8_t len);

/**
 * @brief Erase all flash sectors associated with STG config.
 * @note All STG config data will be lost and is NOT recoverable.
 * @return 0 if successful, otherwise a negative error code. 
 */
int stg_config_erase_all(void);

#endif /* STG_CONFIG_H */
