
/*
* Copyright (c) 2022 Nofence AS
*/

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/flash.h>
#include <storage/flash_map.h>
#include <fs/nvs.h>
#include <logging/log.h>
#include <pm_config.h>

#include "stg_config.h"
#include "storage.h"

LOG_MODULE_REGISTER(stg_config, CONFIG_STG_CONFIG_LOG_LEVEL);

/* Config parameter types */
enum {
    STG_INVALID_PARAM_TYPE = 0,
    STG_U8_PARAM_TYPE,
    STG_U16_PARAM_TYPE,
    STG_U32_PARAM_TYPE,
    STG_STR_PARAM_TYPE,
    STG_BLOB_PARAM_TYPE
}stg_param_type;

static const struct device* mp_device;
static const struct flash_area* mp_flash_area;
static struct nvs_fs m_file_system;
static bool m_initialized = false;

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
    if (m_initialized != true)
    {
        flash_area_id = FLASH_AREA_ID(config_partition);

        err = flash_area_open(flash_area_id, &mp_flash_area);
        if (err != 0) 
        {
            LOG_ERR("STG Config, unable to open flash area");
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
            LOG_ERR("STG Config, unable to get device");
            return -ENODEV;
        }

        m_file_system.offset = (int)mp_flash_area->fa_off;
        m_file_system.sector_size = SECTOR_SIZE;
        m_file_system.sector_count = PM_CONFIG_PARTITION_SIZE / SECTOR_SIZE;

        err = nvs_init(&m_file_system, mp_device->name);
        if (err != 0) 
        {
            LOG_ERR("STG Config, failed to initialize NVS storage");
            return err;
        }
        m_initialized = true;
    }
    return 0;
}

int stg_config_u8_read(stg_config_param_id_t id, uint8_t *value)
{
	int ret;
    if (m_initialized != true)
    {
        LOG_WRN("STG Config not initialized");
        return -ENODEV;
    }

    ret = is_valid_id(id);
    if (ret != STG_U8_PARAM_TYPE)
    {
        LOG_WRN("STG u8 read, invalid id (%d), access denied", (int)id);
        return -ENOMSG;
    }

    uint8_t val;
    ret = nvs_read(&m_file_system, (uint16_t)id, &val, sizeof(uint8_t));
    if (ret == -ENOENT) 
    {
        /* Return u8 max if id-data pair does not exist */
        val = UINT8_MAX; 
    }
    else if ((ret < 0) && (ret != -ENOENT))
    {
        LOG_ERR("STG u8 read, failed to read storage at id %d", (int)id);
        return ret;
    }
    *value = val;
    LOG_DBG("Read: Id=%d, Value=%d", (int)id, *value);
    return 0;
}

int stg_config_u8_write(stg_config_param_id_t id, const uint8_t value)
{
	int ret;
    if (m_initialized != true)
    {
        LOG_WRN("STG Config not initialized");
        return -ENODEV;
    }

    ret = is_valid_id(id);
    if (ret != STG_U8_PARAM_TYPE)
    {
        LOG_WRN("STG u8 write, invalid id (%d), access denied", (int)id);
        return -ENOMSG;
    }

    ret = nvs_write(&m_file_system, (uint16_t)id, &value, sizeof(uint8_t));
    if (ret < 0)
    {
        LOG_ERR("STG u8 write, failed write to storage at id %d", (int)id);
        return ret;
    }
    LOG_DBG("Write: Id=%d, Value=%d", (int)id, value);
    return 0;
}


int stg_config_u16_read(stg_config_param_id_t id, uint16_t *value)
{
	int ret;
    if (m_initialized != true)
    {
        LOG_WRN("STG Config not initialized");
        return -ENODEV;
    }

    ret = is_valid_id(id);
    if (ret != STG_U16_PARAM_TYPE)
    {
        LOG_WRN("STG u16 read, invalid id (%d), access denied", (int)id);
        return -ENOMSG;
    }

    uint16_t val;
    ret = nvs_read(&m_file_system, (uint16_t)id, &val, sizeof(uint16_t));
    if (ret == -ENOENT) 
    {
        /* Return u16 max if id-data pair does not exist */
        val = UINT16_MAX;
    }
	else if ((ret < 0) && (ret != -ENOENT)) 
	{
		LOG_ERR("STG u16 read, failed to read storage at id %d", (int)id);
        return ret;
	}
    *value = val;
    LOG_DBG("Read: Id=%d, Value=%d", (int)id, *value);
    return 0;
}

int stg_config_u16_write(stg_config_param_id_t id, const uint16_t value)
{
	int ret;
    if (m_initialized != true)
    {
        LOG_WRN("STG Config not initialized");
        return -ENODEV;
    }

    ret = is_valid_id(id);
    if (ret != STG_U16_PARAM_TYPE)
    {
        LOG_WRN("STG u16 write, invalid id (%d), access denied", (int)id);
        return -ENOMSG;
    }

    ret = nvs_write(&m_file_system, (uint16_t)id, &value, sizeof(uint16_t));
    if (ret < 0)
    {
        LOG_ERR("STG u16 write, failed write to storage at id %d", (int)id);
        return ret;
    }
    LOG_DBG("Write: Id=%d, Value=%d", (int)id, value);
    return 0;
}

int stg_config_u32_read(stg_config_param_id_t id, uint32_t *value)
{
	int ret;
    if (m_initialized != true)
    {
        LOG_WRN("STG Config not initialized");
        return -ENODEV;
    }

    ret = is_valid_id(id);
    if (ret != STG_U32_PARAM_TYPE)
    {
        LOG_WRN("STG u32 read, invalid id (%d), access denied", (int)id);
        return -ENOMSG;
    }

    uint32_t val;
    ret = nvs_read(&m_file_system, (uint16_t)id, &val, sizeof(uint32_t));
    if (ret == -ENOENT) 
    {
        /* Return u32 max if id-data pair does not exist */
        val = UINT32_MAX;
    }
	else if ((ret < 0) && (ret != -ENOENT)) 
	{
		LOG_ERR("STG u32 read, failed to read storage at id %d", (int)id);
        return ret;
	}
    *value = val;
    LOG_DBG("Read: Id=%d, Value=%d", (int)id, *value);
    return 0;
}

int stg_config_u32_write(stg_config_param_id_t id, const uint32_t value)
{
	int ret;
    if (m_initialized != true)
    {
        LOG_WRN("STG Config not initialized");
        return -ENODEV;
    }

    ret = is_valid_id(id);
    if (ret != STG_U32_PARAM_TYPE)
    {
        LOG_WRN("STG u32 write, invalid id (%d), access denied", (int)id);
        return -ENOMSG;
    }

    ret = nvs_write(&m_file_system, (uint16_t)id, &value, sizeof(uint32_t));
    if (ret < 0)
    {
        LOG_ERR("STG u32 write, failed write to storage at id %d", (int)id);
        return ret;
    }
    LOG_DBG("Write: Id=%d, Value=%d", (int)id, value);
    return 0;
}

int stg_config_str_read(stg_config_param_id_t id, char *str, uint8_t *len)
{
    int ret;
    if (m_initialized != true)
    {
        LOG_WRN("STG Config not initialized");
        return -ENODEV;
    }

    ret = is_valid_id(id);
    if (ret != STG_STR_PARAM_TYPE)
    {
        LOG_WRN("STG str write, invalid id (%d), access denied", (int)id);
        return -ENOMSG;
    }

    char buff[(id == STG_STR_HOST_PORT) ? STG_CONFIG_HOST_PORT_BUF_LEN : 0];
    if (sizeof(buff) <= 0)
    {
        LOG_WRN("STG str read, unknown data size (%d)", sizeof(buff));
        return -ENOMSG;
    }
    memset(buff, 0, sizeof(buff));

    ret = nvs_read(&m_file_system, (uint16_t)id, &buff, sizeof(buff));
    if (ret == -ENOENT)
    {
        /* Return empty string if id-data pair does not exist */
        strcpy(buff, "\0");
    }
    else if ((ret < 0) && (ret != -ENOENT))
    {
        LOG_ERR("STG str read, failed read to storage at id %d", (int)id);
        return ret;
    }
    memcpy(str, buff, sizeof(buff));
    *len = strlen(buff);
    LOG_DBG("Read: Id=%d, Length=%d, Data=%s", (int)id, *len, str);
    return 0;
}

int stg_config_str_write(stg_config_param_id_t id, const char *str, 
        const uint8_t len)
{
    int ret;
    if (m_initialized != true)
    {
        LOG_WRN("STG Config not initialized");
        return -ENODEV;
    }

    ret = is_valid_id(id);
    if (ret != STG_STR_PARAM_TYPE)
    {
        LOG_WRN("STG str write, invalid id (%d), access denied", (int)id);
        return -ENOMSG;
    }
    if ((id == STG_STR_HOST_PORT) && (len > STG_CONFIG_HOST_PORT_BUF_LEN))
    {
        LOG_WRN("STG str write, incorrect size (%d) for id (%d)", len, (int)id);
        return -EOVERFLOW;
    }

    char buff[(id == STG_STR_HOST_PORT) ? STG_CONFIG_HOST_PORT_BUF_LEN : 0];
    if (sizeof(buff) <= 0)
    {
        LOG_WRN("STG str write, unknown data size (%d)", sizeof(buff));
        return -ENOMSG;
    }
    memset(buff, 0, sizeof(buff));
    memcpy(buff, str, sizeof(buff));
    buff[sizeof(buff)-1] = '\0'; //Ensure termination

    ret = nvs_write(&m_file_system, (uint16_t)id, buff, sizeof(buff));
    if (ret < 0)
    {
        LOG_ERR("STG str write, failed write to storage at id %d", (int)id);
        return ret;
    }
    LOG_DBG("Read: Id=%d, Length=%d, Data=%s", (int)id, strlen(buff), buff);
    return 0;
}

int stg_config_blob_read(stg_config_param_id_t id, uint8_t *arr, uint8_t *len)
{
    int ret;
    if (m_initialized != true)
    {
        LOG_WRN("STG Config not initialized");
        return -ENODEV;
    }

    ret = is_valid_id(id);
    if (ret != STG_BLOB_PARAM_TYPE)
    {
        LOG_WRN("STG blob write, invalid id (%d), access denied", (int)id);
        return -ENOMSG;
    }

    uint8_t buff[(id == STG_BLOB_BLE_KEY) ? STG_CONFIG_BLE_SEC_KEY_LEN : 0];
    if (sizeof(buff) <= 0)
    {
        LOG_WRN("STG blob read, unknown data size (%d)", sizeof(buff));
        return -ENOMSG;
    }
    memset(buff, 0, sizeof(buff));

    ret = nvs_read(&m_file_system, (uint16_t)id, &buff, sizeof(buff));
    if (ret == -ENOENT)
    {
        /* Return nothing if blob id-data pair does not exist */
        return 0;
    }
    else if ((ret < 0) && (ret != -ENOENT))
    {
        LOG_ERR("STG blob read, failed read to storage at id %d", (int)id);
        return ret;
    }
    memcpy(arr, buff, sizeof(buff));
    *len = sizeof(buff);

    LOG_DBG("Read: Id=%d, Length=%d, Data=0x%X%X%X%X%X%X%X%X", (int)id, *len, 
        arr[0], arr[1], arr[2], arr[3], arr[4], arr[5], arr[6], arr[7]);

    return 0;
}

int stg_config_blob_write(stg_config_param_id_t id, const uint8_t *arr, 
        const uint8_t len)
{
    int ret;
    if (m_initialized != true)
    {
        LOG_WRN("STG Config not initialized");
        return -ENODEV;
    }

    ret = is_valid_id(id);
    if (ret != STG_BLOB_PARAM_TYPE)
    {
        LOG_WRN("STG blob write, invalid id (%d), access denied", (int)id);
        return -ENOMSG;
    }
    if ((id == STG_BLOB_BLE_KEY) && (len > STG_CONFIG_BLE_SEC_KEY_LEN))
    {
        LOG_WRN("STG blb write, incorrect size (%d) for id (%d)", len, (int)id);
        return -EOVERFLOW;
    }

    ret = nvs_write(&m_file_system, (uint16_t)id, arr, len);
    if (ret < 0)
    {
        LOG_ERR("STG blob write, failed write to storage at id %d", (int)id);
        return ret;
    }

    LOG_DBG("Read: Id=%d, Length=%d, Data=0x%X%X%X%X%X%X%X%X", (int)id, len, 
        arr[0], arr[1], arr[2], arr[3], arr[4], arr[5], arr[6], arr[7]);

    return 0;
}

int stg_config_erase_all()
{
    int ret;
    if (m_initialized != true)
    {
        LOG_WRN("STG Config not initialized");
        return -ENODEV;
    }

    ret = flash_area_erase(mp_flash_area, (uint32_t)mp_flash_area->fa_off, 
            (uint32_t)mp_flash_area->fa_size);
    if (ret != 0)
    {
        LOG_ERR("STG Config, unable to erase flash sectors, err %d", ret);
        return ret;
    }
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
        case STG_STR_HOST_PORT:
        {
            param_type = STG_STR_PARAM_TYPE;
            break;
        }
        case STG_BLOB_BLE_KEY:
        {
            param_type = STG_BLOB_PARAM_TYPE;
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
