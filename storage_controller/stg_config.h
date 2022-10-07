/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef STG_CONFIG_H
#define STG_CONFIG_H

#include <zephyr.h>

/**
 * @brief Identifiers for flash configuration parameters.
 */
typedef enum {
    /* NB! Do NOT change or reorder the identifiers without a sector erase */
    STG_U8_WARN_MAX_DURATION = 0,
	STG_U8_WARN_MIN_DURATION,
	STG_U8_PAIN_CNT_DEF_ESCAPED,
	STG_U8_COLLAR_MODE,
	STG_U8_FENCE_STATUS,
	STG_U8_COLLAR_STATUS,
	STG_U8_TEACH_MODE_FINISHED,
	STG_U8_EMS_PROVIDER,
	STG_U8_PRODUCT_RECORD_REV,
	STG_U8_BOM_MEC_REV,
	STG_U8_BOM_PCB_REV,
	STG_U8_HW_VERSION,
	STG_U8_KEEP_MODE,
	STG_U8_RESET_REASON,
	STG_U16_ACC_SIGMA_NOACTIVITY_LIMIT,
	STG_U16_OFF_ANIMAL_TIME_LIMIT_SEC,
	STG_U16_ACC_SIGMA_SLEEP_LIMIT,
	STG_U16_ZAP_CNT_TOT,
	STG_U16_ZAP_CNT_DAY,
	STG_U16_PRODUCT_TYPE,
    STG_U32_UID, 
    STG_U32_WARN_CNT_TOT,
    STG_STR_HOST_PORT,
    STG_STR_BLE_KEY
}stg_config_param_id_t;

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
int stg_config_u8_write(stg_config_param_id_t id, uint8_t value);

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
int stg_config_u16_write(stg_config_param_id_t id, uint16_t value);

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
int stg_config_u32_write(stg_config_param_id_t id, uint32_t value);

/**
 * @brief Reads the string config parameter as specified by the identifier.
 * @param id The identifier of the config parameter (See stg_config_param_id_t).
 * @param str Pointer to the data buffer.
 * @param len The length of the data.
 * @return 0 if successful, otherwise a negative error code. 
 */
int stg_config_str_read(stg_config_param_id_t id, uint8_t *str, size_t len);

/**
 * @brief Writes to the string config parameter as specified by the identifier.
 * @param id The identifier of the config parameter (See stg_config_param_id_t).
 * @param str Pointer to the data buffer.
 * @param len The length of the data.
 * @return 0 if successful, otherwise a negative error code. 
 */
int stg_config_str_write(stg_config_param_id_t id, uint8_t *str, size_t len);

#endif /* STG_CONFIG_H */
