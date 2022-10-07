
/*
* Copyright (c) 2022 Nofence AS
*/

#include <drivers/flash.h>
#include <storage/flash_map.h>
#include <fs/nvs.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>
#include <pm_config.h>

#include "stg_config.h"

LOG_MODULE_REGISTER(stg_config, 4);

#define STG_CONFIG_HOST_PORT_BUF_LEN 24
#define STG_CONFIG_BLE_SEC_KEY_LEN 8

enum {
    STG_INVALID_PARAM_TYPE = 0,
    STG_U8_PARAM_TYPE,
    STG_U16_PARAM_TYPE,
    STG_U32_PARAM_TYPE,
    STG_STR_PARAM_TYPE
}stg_param_type;

static const struct device* mp_device;
static const struct flash_area* mp_flash_area;
static struct nvs_fs m_file_system;
// static bool m_initialized = false;

/**
 * @brief Check whether the Id-data pair identifier is valid.
 * @param param_id The Id-data pair identifier.
 * @return Returns the parameter type if the id is valid (See stg_param_type).
 */
int is_valid_id(stg_config_param_id_t param_id);


int stg_config_init(void)
{
    int err;
	int flash_area_id;

	flash_area_id = FLASH_AREA_ID(config_partition);

	err = flash_area_open(flash_area_id, &mp_flash_area);
	if (err != 0) 
    {
        LOG_ERR("Unable to open config storage flash area");
		return err;
	}

    LOG_DBG("STG Config: AreaID(%d), FaID(%d), FaOff(%d), FaSize(%d)", 
        flash_area_id, 
        (uint8_t)mp_flash_area->fa_id, 
        (int)mp_flash_area->fa_off, 
        (int)mp_flash_area->fa_size);

	mp_device = device_get_binding(mp_flash_area->fa_dev_name);
	if (mp_device == NULL) 
    {
		LOG_ERR("Unable to get config storage device");
        return -ENODEV;
	}

	m_file_system.offset = (int)mp_flash_area->fa_off;
	m_file_system.sector_size = 4096;
	m_file_system.sector_count = 2;

	err = nvs_init(&m_file_system, mp_device->name);
	if (err != 0) 
    {
		LOG_ERR("Fail to initialize config storage");
        return err;
	}
    // m_initialized = true;
    return 0;
}

int stg_config_u8_read(stg_config_param_id_t type, uint8_t *value)
{
	int ret;
    ret = is_valid_id(type);
    if (ret != STG_U8_PARAM_TYPE)
    {
        LOG_WRN("STG u8 read, access denied, invalid id (%d)", (int)type);
        return -EACCES;
    }

    uint8_t val;
    ret = nvs_read(&m_file_system, (uint16_t)type, &val, sizeof(uint8_t));
	if (ret < 0) 
	{
		LOG_ERR("STG u8 read, failed to read storage at id %d", (int)type);
        return ret;
	} 
    *value = val;
    LOG_DBG("STG u8 read, Id:%d, Value:%d", (int)type, *value);
    return 0;
}

int stg_config_u8_write(stg_config_param_id_t type, uint8_t value)
{
	int ret;
    ret = is_valid_id(type);
    if (ret != STG_U8_PARAM_TYPE)
    {
        LOG_WRN("STG u8 write, access denied, invalid id (%d)", (int)type);
        return -EACCES;
    }

    ret = nvs_write(&m_file_system, (uint16_t)type, &value, sizeof(uint8_t));
    if (ret < 0)
    {
        LOG_ERR("STG u8 write, failed write to storage at id %d", (int)type);
        return ret;
    }
    LOG_DBG("STG u8 write, Id:%d, Value:%d", (int)type, value);
    return 0;
}


int stg_config_u16_read(stg_config_param_id_t type, uint16_t *value)
{
	int ret;
    ret = is_valid_id(type);
    if (ret != STG_U16_PARAM_TYPE)
    {
        LOG_WRN("STG u16 read, access denied, invalid id (%d)", (int)type);
        return -EACCES;
    }

    uint16_t val;
    ret = nvs_read(&m_file_system, (uint16_t)type, &val, sizeof(uint16_t));
	if (ret < 0) 
	{
		LOG_ERR("STG u16 read, failed to read storage at id %d", (int)type);
        return ret;
	}
    *value = val;
    LOG_DBG("STG u16 read, Id:%d, Value:%d", (int)type, *value);
    return 0;
}

int stg_config_u16_write(stg_config_param_id_t type, uint16_t value)
{
	int ret;
    ret = is_valid_id(type);
    if (ret != STG_U16_PARAM_TYPE)
    {
        LOG_WRN("STG u16 write, access denied, invalid id (%d)", (int)type);
        return -EACCES;
    }

    ret = nvs_write(&m_file_system, (uint16_t)type, &value, sizeof(uint16_t));
    if (ret < 0)
    {
        LOG_ERR("STG u16 write, failed write to storage at id %d", (int)type);
        return ret;
    }
    LOG_DBG("STG u8 write, Id:%d, Value:%d", (int)type, value);
    return 0;
}

int stg_config_u32_read(stg_config_param_id_t type, uint32_t *value)
{
	int ret;
    ret = is_valid_id(type);
    if (ret != STG_U32_PARAM_TYPE)
    {
        LOG_WRN("STG u32 read, access denied, invalid id (%d)", (int)type);
        return -EACCES;
    }

    uint32_t val;
    ret = nvs_read(&m_file_system, (uint16_t)type, &val, sizeof(uint32_t));
	if (ret < 0) 
	{
		LOG_ERR("STG u32 read, failed to read storage at id %d", (int)type);
        return ret;
	}
    *value = val;
    LOG_DBG("STG u32 read, Id:%d, Value:%d", (int)type, *value);
    return 0;
}

int stg_config_u32_write(stg_config_param_id_t type, uint32_t value)
{
	int ret;
    ret = is_valid_id(type);
    if (ret != STG_U32_PARAM_TYPE)
    {
        LOG_WRN("STG u32 write, access denied, invalid id (%d)", (int)type);
        return -EACCES;
    }

    ret = nvs_write(&m_file_system, (uint16_t)type, &value, sizeof(uint32_t));
    if (ret < 0)
    {
        LOG_ERR("STG u32 write, failed write to storage at id %d", (int)type);
        return ret;
    }
    LOG_DBG("STG u32 write, Id:%d, Value:%d", (int)type, value);
    return 0;
}

int stg_config_str_read(stg_config_param_id_t param, uint8_t *str, size_t len)
{
    return 0;
}

int stg_config_str_write(stg_config_param_id_t param, uint8_t *str, size_t len)
{
    return 0;
}

int is_valid_id(stg_config_param_id_t param_id)
{
    int param_type;
    switch((stg_config_param_id_t)param_id) 
    {
        case STG_U8_WARN_MAX_DURATION:
        case STG_U8_WARN_MIN_DURATION:
        case STG_U8_PAIN_CNT_DEF_ESCAPED:
        case STG_U8_COLLAR_MODE:
        case STG_U8_FENCE_STATUS:
        case STG_U8_COLLAR_STATUS:
        case STG_U8_TEACH_MODE_FINISHED:
        case STG_U8_EMS_PROVIDER:
        case STG_U8_PRODUCT_RECORD_REV:
        case STG_U8_BOM_MEC_REV:
        case STG_U8_BOM_PCB_REV:
        case STG_U8_HW_VERSION:
        case STG_U8_KEEP_MODE:
        case STG_U8_RESET_REASON:
        {
            param_type = STG_U8_PARAM_TYPE;
            break;
        }
        case STG_U16_ACC_SIGMA_NOACTIVITY_LIMIT:
        case STG_U16_OFF_ANIMAL_TIME_LIMIT_SEC:
        case STG_U16_ACC_SIGMA_SLEEP_LIMIT:
        case STG_U16_ZAP_CNT_TOT:
        case STG_U16_ZAP_CNT_DAY:
        case STG_U16_PRODUCT_TYPE:
        {
            param_type = STG_U16_PARAM_TYPE;
            break;
        }
        case STG_U32_UID:
        case STG_U32_WARN_CNT_TOT: 
        {
            param_type = STG_U32_PARAM_TYPE;
            break;
        }
        default:
        {
            param_type = STG_INVALID_PARAM_TYPE;
            break;
        }
    }
    return param_type;
}







// #define EEPROM_DEFAULT_VALUE_8_T 0xFF
// #define EEPROM_DEFAULT_VALUE_16_T 0xFFFF
// #define EEPROM_DEFAULT_VALUE_32_T 0xFFFFFFFF

// /* EEPROM device pointer */
// const struct device *m_p_device;

// void eep_init(const struct device *dev)
// {
// 	m_p_device = dev;
// }

// int eep_write_host_port(const char *host_port)
// {
// 	if (strlen(host_port) > EEP_HOST_PORT_BUF_SIZE - 1) {
// 		return -EOVERFLOW;
// 	}
// 	/* Note, write the string including null-terminator */
// 	return eeprom_write(m_p_device, offsetof(struct eemem, eep_host_port),
// 			    host_port, strlen(host_port) + 1);
// }

// int eep_read_host_port(char *host_port, size_t bufsize)
// {
// 	if (bufsize < EEP_HOST_PORT_BUF_SIZE) {
// 		return -EOVERFLOW;
// 	}
// 	int ret = eeprom_read(m_p_device, offsetof(struct eemem, eep_host_port),
// 			      host_port, EEP_HOST_PORT_BUF_SIZE);
// 	if (ret == 0) {
// 		host_port[EEP_HOST_PORT_BUF_SIZE - 1] = '\0';
// 	}
// 	return ret;
// }

// /** @todo Create macros instead to directly convert enum to offset. */
// int eep_uint8_read(eep_uint8_enum_t field, uint8_t *value)
// {
// 	off_t offset;
// 	int ret = 0;

// 	switch (field) {
// 	case EEP_WARN_MAX_DURATION: {
// 		offset = offsetof(struct eemem, eep_warn_max_duration);

// 		/* Check for default values, i.e value read is 0xFF,
// 		 * set to defualt WARN_MAX value. 
// 		 */
// 		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
// 		*value = *value == EEPROM_DEFAULT_VALUE_8_T ?
// 				 WARN_MAX_DURATION :
// 				 *value;
// 		LOG_INF("Read EEP_WARN_MAX_DURATION %i", *value);
// 		break;
// 	}
// 	case EEP_WARN_MIN_DURATION: {
// 		offset = offsetof(struct eemem, eep_warn_min_duration);

// 		/* Check for default values, i.e value read is 0xFF,
// 		 * set to defualt WARN_MIN value. 
// 		 */
// 		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
// 		*value = *value == EEPROM_DEFAULT_VALUE_8_T ?
// 				 WARN_MIN_DURATION :
// 				 *value;
// 		LOG_INF("Read EEP_WARN_MIN_DURATION %i", *value);
// 		break;
// 	}
// 	case EEP_PAIN_CNT_DEF_ESCAPED: {
// 		offset = offsetof(struct eemem, eep_warn_min_duration);

// 		/* Check for default values, i.e value read is 0xFF,
// 		 * set to default CNT_DEF_ESCAPED value. 
// 		 */
// 		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
// 		*value = *value == EEPROM_DEFAULT_VALUE_8_T ?
// 				 PAIN_CNT_DEF_ESCAPED :
// 				 *value;
// 		LOG_INF("Read EEP_PAIN_CNT_DEF_ESCAPED %i", *value);
// 		break;
// 	}
// 	case EEP_FENCE_STATUS: {
// 		offset = offsetof(struct eemem, eep_fence_status);

// 		/* Check for default values, i.e value read is 0xFF,
// 		 * set to defualt FenceStatus_FenceStatus_UNKNOWN. 
// 		 */
// 		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
// 		*value = *value == EEPROM_DEFAULT_VALUE_8_T ?
// 				 FenceStatus_FenceStatus_UNKNOWN :
// 				 *value;
// 		LOG_INF("Read EEP_FENCE_STATUS %i", *value);
// 		break;
// 	}
// 	case EEP_COLLAR_MODE: {
// 		offset = offsetof(struct eemem, eep_collar_mode);

// 		/* Check for default values, i.e value read is 0xFF,
// 		 * set to defualt Mode_Mode_UNKNOWN. 
// 		 */
// 		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
// 		*value = *value == EEPROM_DEFAULT_VALUE_8_T ?
// 				 Mode_Mode_UNKNOWN :
// 				 *value;
// 		LOG_INF("Read EEP_COLLAR_MODE %i", *value);
// 		break;
// 	}
// 	case EEP_COLLAR_STATUS: {
// 		offset = offsetof(struct eemem, eep_collar_status);

// 		/* Check for default values, i.e value read is 0xFF,
// 		 * set to defualt CollarStatus_CollarStatus_UNKNOWN. 
// 		 */
// 		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
// 		*value = *value == EEPROM_DEFAULT_VALUE_8_T ?
// 				 CollarStatus_CollarStatus_UNKNOWN :
// 				 *value;
// 		LOG_INF("Read EEP_COLLAR_STATUS %i", *value);
// 		break;
// 	}
// 	case EEP_TEACH_MODE_FINISHED: {
// 		offset = offsetof(struct eemem, eep_teach_mode_finished);

// 		/* Check for default values, i.e value read is 0xFF,
// 		 * set to defualt 0, not finished. 
// 		 */
// 		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
// 		*value = *value == EEPROM_DEFAULT_VALUE_8_T ? 0 : *value;
// 		LOG_INF("Read teach mode finished %i", *value);
// 		break;
// 	}
// 	case EEP_KEEP_MODE: {
// 		offset = offsetof(struct eemem, eep_keep_mode);
// 		/* Check for default values, i.e value read is 0xFF,
// 		 * set to defualt 0, not finished. 
// 		 */
// 		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
// 		*value = *value == EEPROM_DEFAULT_VALUE_8_T ? 0 : *value;
// 		LOG_INF("Read keep mode %i", *value);
// 		break;
// 	}
// 	case EEP_EMS_PROVIDER: {
// 		offset = offsetof(struct eemem, eep_ems_provider);

// 		/* Check for default values, i.e value read is 0xFF,
// 		 * set to defualt 0, not finished. 
// 		 */
// 		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
// 		*value = *value == EEPROM_DEFAULT_VALUE_8_T ? 0 : *value;
// 		LOG_INF("Set EMS provider to %i", *value);
// 		break;
// 	}
// 	case EEP_PRODUCT_RECORD_REV: {
// 		offset = offsetof(struct eemem, eep_product_record_rev);

// 		/* Check for default values, i.e value read is 0xFF,
// 		 * set to defualt 0, not finished. 
// 		 */
// 		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
// 		*value = *value == EEPROM_DEFAULT_VALUE_8_T ? 0 : *value;
// 		LOG_INF("Set product record rev to %i", *value);
// 		break;
// 	}
// 	case EEP_BOM_MEC_REV: {
// 		offset = offsetof(struct eemem, eep_bom_mec_rev);

// 		/* Check for default values, i.e value read is 0xFF,
// 		 * set to defualt 0, not finished. 
// 		 */
// 		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
// 		*value = *value == EEPROM_DEFAULT_VALUE_8_T ? 0 : *value;
// 		LOG_INF("Set BOM mec rev to %i", *value);
// 		break;
// 	}
// 	case EEP_BOM_PCB_REV: {
// 		offset = offsetof(struct eemem, eep_bom_pcb_rev);

// 		/* Check for default values, i.e value read is 0xFF,
// 		 * set to defualt 0, not finished. 
// 		 */
// 		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
// 		*value = *value == EEPROM_DEFAULT_VALUE_8_T ? 0 : *value;
// 		LOG_INF("Set BOM pcb rev to %i", *value);
// 		break;
// 	}
// 	case EEP_HW_VERSION: {
// 		offset = offsetof(struct eemem, eep_hw_version);

// 		/* Check for default values, i.e value read is 0xFF,
// 		 * set to defualt 0, not finished. 
// 		 */
// 		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
// 		*value = *value == EEPROM_DEFAULT_VALUE_8_T ? 0 : *value;
// 		LOG_INF("Set hw version to %i", *value);
// 		break;
// 	}
// 	case EEP_RESET_REASON: {
// 		offset = offsetof(struct eemem, eep_reset_reason);

// 		/* Check for default values, i.e value read is 0xFF,
// 		 * set to defualt 0, not finished. 
// 		 */
// 		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
// 		*value = *value == EEPROM_DEFAULT_VALUE_8_T ? 0 : *value;
// 		LOG_INF("Read EEP_RESET_REASON %i", *value);
// 		break;
// 	}
// 	default: {
// 		return -EINVAL;
// 	}
// 	}

// 	return ret;
// }

// int eep_uint8_write(eep_uint8_enum_t field, uint8_t value)
// {
// 	off_t offset;

// 	switch (field) {
// 	case EEP_WARN_MAX_DURATION: {
// 		offset = offsetof(struct eemem, eep_warn_max_duration);
// 		break;
// 	}
// 	case EEP_WARN_MIN_DURATION: {
// 		offset = offsetof(struct eemem, eep_warn_min_duration);
// 		break;
// 	}
// 	case EEP_PAIN_CNT_DEF_ESCAPED: {
// 		offset = offsetof(struct eemem, eep_pain_cnt_def_escaped);
// 		break;
// 	}
// 	case EEP_COLLAR_MODE: {
// 		offset = offsetof(struct eemem, eep_collar_mode);
// 		break;
// 	}
// 	case EEP_FENCE_STATUS: {
// 		offset = offsetof(struct eemem, eep_fence_status);
// 		break;
// 	}
// 	case EEP_COLLAR_STATUS: {
// 		offset = offsetof(struct eemem, eep_collar_status);
// 		break;
// 	}
// 	case EEP_TEACH_MODE_FINISHED: {
// 		offset = offsetof(struct eemem, eep_teach_mode_finished);
// 		break;
// 	}
// 	case EEP_EMS_PROVIDER: {
// 		offset = offsetof(struct eemem, eep_ems_provider);
// 		break;
// 	}
// 	case EEP_PRODUCT_RECORD_REV: {
// 		offset = offsetof(struct eemem, eep_product_record_rev);
// 		break;
// 	}
// 	case EEP_BOM_MEC_REV: {
// 		offset = offsetof(struct eemem, eep_bom_mec_rev);
// 		break;
// 	}
// 	case EEP_BOM_PCB_REV: {
// 		offset = offsetof(struct eemem, eep_bom_pcb_rev);
// 		break;
// 	}
// 	case EEP_HW_VERSION: {
// 		offset = offsetof(struct eemem, eep_hw_version);
// 		break;
// 	}
// 	case EEP_KEEP_MODE: {
// 		offset = offsetof(struct eemem, eep_keep_mode);
// 		break;
// 	}
// 	case EEP_RESET_REASON: {
// 		offset = offsetof(struct eemem, eep_reset_reason);
// 		break;
// 	}
// 	default: {
// 		return -EINVAL;
// 	}
// 	}

// 	return eeprom_write(m_p_device, offset, &value, sizeof(value));
// }

// int eep_uint16_read(eep_uint16_enum_t field, uint16_t *value)
// {
// 	off_t offset;
// 	int ret = 0;

// 	switch (field) {
// 	case EEP_ACC_SIGMA_NOACTIVITY_LIMIT: {
// 		offset = offsetof(struct eemem, eep_acc_sigma_noactivity_limit);

// 		/* Check for default values, i.e value read is 0xFFFF,
// 		 * set to defualt 0 total zaps. 
// 		 */
// 		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
// 		*value = *value == EEPROM_DEFAULT_VALUE_16_T ?
// 				 ACC_SIGMA_NOACTIVITY_LIMIT_DEFAULT :
// 				 *value;
// 		LOG_INF("Set EEP_ACC_SIGMA_NOACTIVITY_LIMIT to %i", *value);
// 		break;
// 	}
// 	case EEP_OFF_ANIMAL_TIME_LIMIT_SEC: {
// 		offset = offsetof(struct eemem, eep_off_animal_time_limit);

// 		/* Check for default values, i.e value read is 0xFFFF,
// 		 * set to defualt 0 total zaps. 
// 		 */
// 		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
// 		*value = *value == EEPROM_DEFAULT_VALUE_16_T ?
// 				 OFF_ANIMAL_TIME_LIMIT_SEC_DEFAULT :
// 				 *value;
// 		LOG_INF("Set EEP_OFF_ANIMAL_TIME_LIMIT_SEC to %i", *value);
// 		break;
// 	}
// 	case EEP_ACC_SIGMA_SLEEP_LIMIT: {
// 		offset = offsetof(struct eemem, eep_acc_sigma_sleep_limit);

// 		/* Check for default values, i.e value read is 0xFFFF,
// 		 * set to defualt 0 total zaps. 
// 		 */
// 		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
// 		*value = *value == EEPROM_DEFAULT_VALUE_16_T ?
// 				 ACC_SIGMA_SLEEP_LIMIT_DEFAULT :
// 				 *value;
// 		LOG_INF("Set EEP_ACC_SIGMA_SLEEP_LIMIT to %i", *value);
// 		break;
// 	}
// 	case EEP_ZAP_CNT_TOT: {
// 		offset = offsetof(struct eemem, eep_zap_cnt_tot);

// 		/* Check for default values, i.e value read is 0xFFFF,
// 		 * set to defualt 0 total zaps. 
// 		 */
// 		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
// 		*value = *value == EEPROM_DEFAULT_VALUE_16_T ? 0 : *value;
// 		LOG_INF("Set EEP_ZAP_CNT_TOT to %i", *value);
// 		break;
// 	}
// 	case EEP_ZAP_CNT_DAY: {
// 		offset = offsetof(struct eemem, eep_zap_cnt_day);

// 		/* Check for default values, i.e value read is 0xFFFF,
// 		 * set to defualt 0 total zaps. 
// 		 */
// 		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
// 		*value = *value == EEPROM_DEFAULT_VALUE_16_T ? 0 : *value;
// 		LOG_INF("Set EEP_ZAP_CNT_DAY to %i", *value);
// 		break;
// 	}
// 	case EEP_PRODUCT_TYPE: {
// 		offset = offsetof(struct eemem, eep_product_type);

// 		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
// 		LOG_INF("Set EEP_PRODUCT_TYPE to %i", *value);
// 		break;
// 	}
// 	default: {
// 		return -EINVAL;
// 	}
// 	}

// 	return ret;
// }

// int eep_uint16_write(eep_uint16_enum_t field, uint16_t value)
// {
// 	off_t offset;

// 	switch (field) {
// 	case EEP_ACC_SIGMA_NOACTIVITY_LIMIT: {
// 		offset = offsetof(struct eemem, eep_acc_sigma_noactivity_limit);
// 		break;
// 	}
// 	case EEP_OFF_ANIMAL_TIME_LIMIT_SEC: {
// 		offset = offsetof(struct eemem, eep_off_animal_time_limit);
// 		break;
// 	}
// 	case EEP_ACC_SIGMA_SLEEP_LIMIT: {
// 		offset = offsetof(struct eemem, eep_acc_sigma_sleep_limit);
// 		break;
// 	}
// 	case EEP_ZAP_CNT_TOT: {
// 		offset = offsetof(struct eemem, eep_zap_cnt_tot);
// 		break;
// 	}
// 	case EEP_ZAP_CNT_DAY: {
// 		offset = offsetof(struct eemem, eep_zap_cnt_day);
// 		break;
// 	}
// 	case EEP_PRODUCT_TYPE: {
// 		offset = offsetof(struct eemem, eep_product_type);
// 		break;
// 	}
// 	default: {
// 		return -EINVAL;
// 	}
// 	}

// 	return eeprom_write(m_p_device, offset, &value, sizeof(value));
// }

// int eep_uint32_read(eep_uint32_enum_t field, uint32_t *value)
// {
// 	off_t offset;
// 	int ret = 0;

// 	switch (field) {
// 	case EEP_UID: {
// 		offset = offsetof(struct eemem, eep_uid);

// 		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
// 		/** @todo Any default IDs to fetch???? What do to? */
// 		//*value = *value == EEPROM_DEFAULT_VALUE_32_T ? 0 : *value;
// 		//LOG_INF("Set EEP_UID to %i", *value);
// 		break;
// 	}
// 	case EEP_WARN_CNT_TOT: {
// 		offset = offsetof(struct eemem, eep_warn_cnt_tot);

// 		/* Check for default values, i.e value read is 0xFFFFFFFF,
// 		 * set to defualt 0 total warnings. 
// 		 */
// 		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
// 		*value = *value == EEPROM_DEFAULT_VALUE_32_T ? 0 : *value;
// 		LOG_INF("Set EEP_WARN_CNT_TOT to %i", *value);
// 		break;
// 	}
// 	default: {
// 		return -EINVAL;
// 	}
// 	}

// 	return ret;
// }

// int eep_uint32_write(eep_uint32_enum_t field, uint32_t value)
// {
// 	off_t offset;

// 	switch (field) {
// 	case EEP_UID: {
// 		offset = offsetof(struct eemem, eep_uid);
// 		break;
// 	}
// 	case EEP_WARN_CNT_TOT: {
// 		offset = offsetof(struct eemem, eep_warn_cnt_tot);
// 		break;
// 	}
// 	default: {
// 		return -EINVAL;
// 	}
// 	}

// 	return eeprom_write(m_p_device, offset, &value, sizeof(value));
// }

// int eep_write_ble_sec_key(uint8_t *ble_sec_key, size_t bufsize)
// {
// 	if (sizeof(bufsize) > EEP_BLE_SEC_KEY_LEN) {
// 		return -EOVERFLOW;
// 	}
// 	/* Note, write the string including null-terminator */
// 	return eeprom_write(m_p_device, offsetof(struct eemem, ble_sec_key),
// 			    ble_sec_key, bufsize);
// }

// int eep_read_ble_sec_key(uint8_t *ble_sec_key, size_t bufsize)
// {
// 	if (bufsize < EEP_BLE_SEC_KEY_LEN) {
// 		return -EOVERFLOW;
// 	}
// 	int ret = eeprom_read(m_p_device, offsetof(struct eemem, ble_sec_key),
// 			      ble_sec_key, EEP_BLE_SEC_KEY_LEN);
// 	return ret;
// }
