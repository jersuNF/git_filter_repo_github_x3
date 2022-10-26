
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
#include "nf_settings.h"

#define COPY_FROM_EEPROM 1

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

#if DT_NODE_HAS_STATUS(DT_ALIAS(eeprom), okay)
/**
 * @brief Copies all config parameters from EEPROM to flash storage.
 * @return 0 if successful, otherwise a negative error code. 
 */
int copy_eeprom_parameters_to_stg_flash();
#endif /* DT_NODE_HAS_STATUS(DT_ALIAS(eeprom), okay) */


int stg_config_init(void)
{
    int err;
	int flash_area_id;
    if (m_initialized != true) {
        flash_area_id = FLASH_AREA_ID(config_partition);

        err = flash_area_open(flash_area_id, &mp_flash_area);
        if (err != 0) {
            LOG_ERR("STG Config, unable to open flash area");
            return err;
        }

        LOG_DBG("STG Config: AreaID(%d), FaID(%d), FaOff(%d), FaSize(%d)", 
            flash_area_id, 
            (uint8_t)mp_flash_area->fa_id, 
            (int)mp_flash_area->fa_off, 
            (int)mp_flash_area->fa_size);

        mp_device = device_get_binding(mp_flash_area->fa_dev_name);
        if (mp_device == NULL) {
            LOG_ERR("STG Config, unable to get device");
            return -ENODEV;
        }

        m_file_system.offset = (int)mp_flash_area->fa_off;
        m_file_system.sector_size = SECTOR_SIZE;
        m_file_system.sector_count = PM_CONFIG_PARTITION_SIZE / SECTOR_SIZE;

        err = nvs_init(&m_file_system, mp_device->name);
        if (err != 0) {
            LOG_ERR("STG Config, failed to initialize NVS storage");
            return err;
        }
        m_initialized = true;

#if DT_NODE_HAS_STATUS(DT_ALIAS(eeprom), okay)
	copy_eeprom_parameters_to_stg_flash();
#endif /* DT_NODE_HAS_STATUS(DT_ALIAS(eeprom), okay) */
    }
    return 0;
}

int stg_config_u8_read(stg_config_param_id_t id, uint8_t *value)
{
	int ret;
    if (m_initialized != true) {
        LOG_WRN("STG Config not initialized");
        return -ENODEV;
    }

    ret = is_valid_id(id);
    if (ret != STG_U8_PARAM_TYPE) {
        LOG_WRN("STG u8 read, invalid id (%d), access denied", (int)id);
        return -ENOMSG;
    }

    uint8_t val;
    ret = nvs_read(&m_file_system, (uint16_t)id, &val, sizeof(uint8_t));
    if (ret == -ENOENT) {
        /* Return u8 max if id-data pair does not exist */
        val = UINT8_MAX; 
    } else if ((ret < 0) && (ret != -ENOENT)) {
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
    if (m_initialized != true) {
        LOG_WRN("STG Config not initialized");
        return -ENODEV;
    }

    ret = is_valid_id(id);
    if (ret != STG_U8_PARAM_TYPE) {
        LOG_WRN("STG u8 write, invalid id (%d), access denied", (int)id);
        return -ENOMSG;
    }

    ret = nvs_write(&m_file_system, (uint16_t)id, &value, sizeof(uint8_t));
    if (ret < 0) {
        LOG_ERR("STG u8 write, failed write to storage at id %d", (int)id);
        return ret;
    }
    LOG_DBG("Write: Id=%d, Value=%d", (int)id, value);
    return 0;
}


int stg_config_u16_read(stg_config_param_id_t id, uint16_t *value)
{
	int ret;
    if (m_initialized != true) {
        LOG_WRN("STG Config not initialized");
        return -ENODEV;
    }

    ret = is_valid_id(id);
    if (ret != STG_U16_PARAM_TYPE) {
        LOG_WRN("STG u16 read, invalid id (%d), access denied", (int)id);
        return -ENOMSG;
    }

    uint16_t val;
    ret = nvs_read(&m_file_system, (uint16_t)id, &val, sizeof(uint16_t));
    if (ret == -ENOENT) {
        /* Return u16 max if id-data pair does not exist */
        val = UINT16_MAX;
    } else if ((ret < 0) && (ret != -ENOENT)) {
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
    if (m_initialized != true) {
        LOG_WRN("STG Config not initialized");
        return -ENODEV;
    }

    ret = is_valid_id(id);
    if (ret != STG_U16_PARAM_TYPE) {
        LOG_WRN("STG u16 write, invalid id (%d), access denied", (int)id);
        return -ENOMSG;
    }

    ret = nvs_write(&m_file_system, (uint16_t)id, &value, sizeof(uint16_t));
    if (ret < 0) {
        LOG_ERR("STG u16 write, failed write to storage at id %d", (int)id);
        return ret;
    }
    LOG_DBG("Write: Id=%d, Value=%d", (int)id, value);
    return 0;
}

int stg_config_u32_read(stg_config_param_id_t id, uint32_t *value)
{
	int ret;
    if (m_initialized != true) {
        LOG_WRN("STG Config not initialized");
        return -ENODEV;
    }

    ret = is_valid_id(id);
    if (ret != STG_U32_PARAM_TYPE) {
        LOG_WRN("STG u32 read, invalid id (%d), access denied", (int)id);
        return -ENOMSG;
    }

    uint32_t val;
    ret = nvs_read(&m_file_system, (uint16_t)id, &val, sizeof(uint32_t));
    if (ret == -ENOENT) {
        /* Return u32 max if id-data pair does not exist */
        val = UINT32_MAX;
    } else if ((ret < 0) && (ret != -ENOENT)) {
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
    if (m_initialized != true) {
        LOG_WRN("STG Config not initialized");
        return -ENODEV;
    }

    ret = is_valid_id(id);
    if (ret != STG_U32_PARAM_TYPE) {
        LOG_WRN("STG u32 write, invalid id (%d), access denied", (int)id);
        return -ENOMSG;
    }

    ret = nvs_write(&m_file_system, (uint16_t)id, &value, sizeof(uint32_t));
    if (ret < 0) {
        LOG_ERR("STG u32 write, failed write to storage at id %d", (int)id);
        return ret;
    }
    LOG_DBG("Write: Id=%d, Value=%d", (int)id, value);
    return 0;
}

int stg_config_str_read(stg_config_param_id_t id, char *str, uint8_t *len)
{
    int ret;
    if (m_initialized != true) {
        LOG_WRN("STG Config not initialized");
        return -ENODEV;
    }

    ret = is_valid_id(id);
    if (ret != STG_STR_PARAM_TYPE) {
        LOG_WRN("STG str write, invalid id (%d), access denied", (int)id);
        return -ENOMSG;
    }

    char buff[(id == STG_STR_HOST_PORT) ? STG_CONFIG_HOST_PORT_BUF_LEN : 0];
    if (sizeof(buff) <= 0) {
        LOG_WRN("STG str read, unknown data size (%d)", sizeof(buff));
        return -ENOMSG;
    }
    memset(buff, 0, sizeof(buff));

    ret = nvs_read(&m_file_system, (uint16_t)id, &buff, sizeof(buff));
    if (ret == -ENOENT) {
        /* Return empty string if id-data pair does not exist */
        strcpy(buff, "\0");
    } else if ((ret < 0) && (ret != -ENOENT)) {
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
    if (m_initialized != true) {
        LOG_WRN("STG Config not initialized");
        return -ENODEV;
    }

    ret = is_valid_id(id);
    if (ret != STG_STR_PARAM_TYPE) {
        LOG_WRN("STG str write, invalid id (%d), access denied", (int)id);
        return -ENOMSG;
    }
    if ((id == STG_STR_HOST_PORT) && (len > STG_CONFIG_HOST_PORT_BUF_LEN)) {
        LOG_WRN("STG str write, incorrect size (%d) for id (%d)", len, (int)id);
        return -EOVERFLOW;
    }

    char buff[(id == STG_STR_HOST_PORT) ? STG_CONFIG_HOST_PORT_BUF_LEN : 0];
    if (sizeof(buff) <= 0) {
        LOG_WRN("STG str write, unknown data size (%d)", sizeof(buff));
        return -ENOMSG;
    }
    memset(buff, 0, sizeof(buff));
    memcpy(buff, str, sizeof(buff));
    buff[sizeof(buff)-1] = '\0'; //Ensure termination

    ret = nvs_write(&m_file_system, (uint16_t)id, buff, sizeof(buff));
    if (ret < 0) {
        LOG_ERR("STG str write, failed write to storage at id %d", (int)id);
        return ret;
    }
    LOG_DBG("Read: Id=%d, Length=%d, Data=%s", (int)id, strlen(buff), buff);
    return 0;
}

int stg_config_blob_read(stg_config_param_id_t id, uint8_t *arr, uint8_t *len)
{
    int ret;
    if (m_initialized != true) {
        LOG_WRN("STG Config not initialized");
        return -ENODEV;
    }

    ret = is_valid_id(id);
    if (ret != STG_BLOB_PARAM_TYPE) {
        LOG_WRN("STG blob write, invalid id (%d), access denied", (int)id);
        return -ENOMSG;
    }

    uint8_t buff[(id == STG_BLOB_BLE_KEY) ? STG_CONFIG_BLE_SEC_KEY_LEN : 0];
    if (sizeof(buff) <= 0) {
        LOG_WRN("STG blob read, unknown data size (%d)", sizeof(buff));
        return -ENOMSG;
    }
    memset(buff, 0, sizeof(buff));

    ret = nvs_read(&m_file_system, (uint16_t)id, &buff, sizeof(buff));
    if (ret == -ENOENT) {
        /* Return nothing if blob id-data pair does not exist */
        return 0;
    } else if ((ret < 0) && (ret != -ENOENT)) {
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
    if (m_initialized != true) {
        LOG_WRN("STG Config not initialized");
        return -ENODEV;
    }

    ret = is_valid_id(id);
    if (ret != STG_BLOB_PARAM_TYPE) {
        LOG_WRN("STG blob write, invalid id (%d), access denied", (int)id);
        return -ENOMSG;
    }
    if ((id == STG_BLOB_BLE_KEY) && (len > STG_CONFIG_BLE_SEC_KEY_LEN)) {
        LOG_WRN("STG blb write, incorrect size (%d) for id (%d)", len, (int)id);
        return -EOVERFLOW;
    }

    ret = nvs_write(&m_file_system, (uint16_t)id, arr, len);
    if (ret < 0) {
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
    if (m_initialized != true) {
        LOG_WRN("STG Config not initialized");
        return -ENODEV;
    }

    ret = flash_area_erase(mp_flash_area, (uint32_t)mp_flash_area->fa_off, 
            (uint32_t)mp_flash_area->fa_size);
    if (ret != 0) {
        LOG_ERR("STG Config, unable to erase flash sectors, err %d", ret);
        return ret;
    }
    return 0;
}

int is_valid_id(stg_config_param_id_t param_id)
{
    int param_type;
    switch((stg_config_param_id_t)param_id) {
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
        case STG_U8_RESET_REASON: {
            param_type = STG_U8_PARAM_TYPE;
            break;
        }
        case STG_U16_ACC_SIGMA_NOACTIVITY_LIMIT:
        case STG_U16_OFF_ANIMAL_TIME_LIMIT_SEC:
        case STG_U16_ACC_SIGMA_SLEEP_LIMIT:
        case STG_U16_ZAP_CNT_TOT:
        case STG_U16_ZAP_CNT_DAY:
        case STG_U16_PRODUCT_TYPE: {
            param_type = STG_U16_PARAM_TYPE;
            break;
        }
        case STG_U32_UID:
        case STG_U32_WARN_CNT_TOT: {
            param_type = STG_U32_PARAM_TYPE;
            break;
        }
        case STG_STR_HOST_PORT: {
            param_type = STG_STR_PARAM_TYPE;
            break;
        }
        case STG_BLOB_BLE_KEY: {
            param_type = STG_BLOB_PARAM_TYPE;
            break;
        }
        default: {
            param_type = STG_INVALID_PARAM_TYPE;
            break;
        }
    }
    return param_type;
}

#if DT_NODE_HAS_STATUS(DT_ALIAS(eeprom), okay)
int copy_eeprom_parameters_to_stg_flash()
{
	int ret = 0;
#ifdef COPY_FROM_EEPROM
	uint8_t eep_stg_copy_val = 0;
	ret = eep_uint8_read(EEP_STG_COPY, &eep_stg_copy_val);
	if (ret != 0) {
		LOG_ERR("EEP(u8) Failed to read id %d, err %d", EEP_STG_COPY, 
			ret);
		return ret;
	}

	/* Do a compare and copy if EEP_STG_COPY flag is 0 */
	int success = 0;
	if (eep_stg_copy_val == 0) {
		uint8_t eep_u8_val;
		uint8_t stg_u8_val;
		for (int i = 0; i < (EEP_RESET_REASON + 1); i++) {
			/* Copy every uint8 parameter from the EEPROM to the 
			 * flash, except the EEP_STG_COPY flag which only reside 
			 * in the actual EEPROM */

			/* Read uint8 EEP value */
			eep_u8_val = 0;
			ret = eep_uint8_read(i, &eep_u8_val);
			if (ret != 0) {
				success = -1;
				LOG_ERR("EEP(u8) Failed to read id %d, err %d", 
					i, ret);
			}

			/* Read uint8 STG value */
			stg_u8_val = 0;
			ret = stg_config_u8_read(i, &stg_u8_val);
			if (ret != 0) {
				success = -1;
				LOG_ERR("STG(u8) Failed to read id %d, err %d", 
					i, ret);
			}
			LOG_INF("EEPvsSTG(u8): Id = %d, EEP = %d, STG = %d", i, 
				eep_u8_val, stg_u8_val);

			/* Compare and copy if eeprom and stg flash values are
			 * not identical */
			if (eep_u8_val != stg_u8_val) {
				ret = stg_config_u8_write(i, eep_u8_val);
				if (ret != 0) {
					success = -1;
					LOG_ERR("STG(u8) Failed to write id %d, err %d", 
						i, ret);
				}

				stg_u8_val = 0;
				ret = stg_config_u8_read(i, &stg_u8_val);
				if (ret != 0) {
					success = -1;
					LOG_ERR("STG(u8) Failed to read id %d, err %d", 
						i, ret);
				}
				LOG_INF("EEPvsSTG(u8): Id = %d, EEP = %d, STG = %d", 
					i, eep_u8_val, stg_u8_val);
			}
		}

		uint16_t eep_u16_val;
		uint16_t stg_u16_val;
		for (int i = 0; i < (EEP_PRODUCT_TYPE + 1); i++) {
			/* Copy every uint16 parameter from the EEPROM to the 
			 * flash */

			/* Read uint16 EEP value */
			eep_u16_val = 0;
			ret = eep_uint16_read(i, &eep_u16_val);
			if (ret != 0) {
				success = -1;
				LOG_ERR("EEP(u16) Failed to read id %d, err %d", 
					i, ret);
			}

			/* Read uint16 STG value */
			stg_u16_val = 0;
			ret = stg_config_u16_read(i+14, &stg_u16_val);
			if (ret != 0) {
				success = -1;
				LOG_ERR("STG(u16) Failed to read id %d(+14), err %d", 
					i, ret);
			}
			LOG_INF("EEPvsSTG(u16): Id = %d(+14), EEP = %d, STG = %d", 
				i, eep_u16_val, stg_u16_val);

			/* Compare and copy if eeprom and stg flash values are
			 * not identical */
			if (eep_u16_val != stg_u16_val) {
				ret = stg_config_u16_write(i+14, eep_u16_val);
				if (ret != 0) {
					success = -1;
					LOG_ERR("STG(u16) Failed to write id %d(+14), err %d", 
						i, ret);
				}

				stg_u16_val = 0;
				ret = stg_config_u16_read(i+14, &stg_u16_val);
				if (ret != 0) {
					success = -1;
					LOG_ERR("STG(u16) Failed to read id %d(+14), err %d", 
						i, ret);
				}
				LOG_INF("EEPvsSTG(u16): Id = %d(+14), EEP = %d, STG = %d", 
					i, eep_u16_val, stg_u16_val);
			}
		}

		uint32_t eep_u32_val;
		uint32_t stg_u32_val;
		for (int i = 0; i < (EEP_WARN_CNT_TOT + 1); i++) {
			/* Copy every uint32 parameter from the EEPROM to the 
			 * flash */

			/* Read uint32 EEP value */
			eep_u32_val = 0;
			ret = eep_uint32_read(i, &eep_u32_val);
			if (ret != 0) {
				success = -1;
				LOG_ERR("EEP(u32) Failed to read id %d, err %d", 
					i, ret);
			}

			/* Read uint32 STG value */
			stg_u32_val = 0;
			ret = stg_config_u32_read(i+20, &stg_u32_val);
			if (ret != 0) {
				success = -1;
				LOG_ERR("STG(u32) Failed to read id %d(+20), err %d", 
					i, ret);
			}
			LOG_INF("EEPvsSTG(u32): Id = %d(+20), EEP = %d, STG = %d", 
				i, eep_u32_val, stg_u32_val);

			/* Compare and copy if eeprom and stg flash values are
			 * not identical */
			if (eep_u32_val != stg_u32_val) {
				ret = stg_config_u32_write(i+20, eep_u32_val);
				if (ret != 0) {
					success = -1;
					LOG_ERR("STG(u32) Failed to write id %d(+20), err %d", 
						i, ret);
				}

				stg_u32_val = 0;
				ret = stg_config_u32_read(i+20, &stg_u32_val);
				if (ret != 0) {
					success = -1;
					LOG_ERR("STG(u32) Failed to read id %d(+20), err %d", 
						i, ret);
				}
				LOG_INF("EEPvsSTG(u32): Id = %d(+20), EEP = %d, STG = %d", 
					i, eep_u32_val, stg_u32_val);
			}
		}

		/* Copy Host Port from the EEPROM to the flash */
		char port[STG_CONFIG_HOST_PORT_BUF_LEN];

		/* Read Host Port from EEPROM */
		ret = eep_read_host_port(&port[0], 
			STG_CONFIG_HOST_PORT_BUF_LEN);
		if (ret != 0) {
			success = -1;
			LOG_ERR("EEP Failed to read host port");
		}
		LOG_INF("EEPvsSTG(Host port): EEP = %s", port);

		ret = stg_config_str_write(STG_STR_HOST_PORT, port, 
			STG_CONFIG_HOST_PORT_BUF_LEN);
		if (ret != 0) {
			success = -1;
			LOG_ERR("STG Failed to write host port to ext flash!");
		}

		char port_read[STG_CONFIG_HOST_PORT_BUF_LEN];
		uint8_t port_len = 0;
		ret = stg_config_str_read(STG_STR_HOST_PORT, port_read, 
			&port_len);
		if (ret != 0) {
			success = -1;
			LOG_ERR("STG Failed to read host port");
		}
		LOG_INF("EEPvsSTG(Host port): STG(%d) = %s", port_len, 
			port_read);


		/* Copy BLE key from the EEPROM to the flash */
		uint8_t ble_key[EEP_BLE_SEC_KEY_LEN];
		memset(ble_key, 0, sizeof(ble_key));

		/* Read BLE Security key from EEPROM */
		ret = eep_read_ble_sec_key(ble_key, EEP_BLE_SEC_KEY_LEN);
		if (ret != 0){
			success = -1;
			LOG_ERR("EEP Failed to read ble key from EEPROM");
		}
		LOG_INF("EEP(BLE key): EEP = 0x%X%X%X%X%X%X%X%X", ble_key[0], 
			ble_key[1], ble_key[2], ble_key[3], ble_key[4], 
			ble_key[5], ble_key[6], ble_key[7]);

		/* Write BLE Security key to flash */
		ret = stg_config_blob_write(STG_BLOB_BLE_KEY, ble_key, 
			EEP_BLE_SEC_KEY_LEN);
		if (ret != 0){
			success = -1;
			LOG_ERR("STG Failed to write ble key to ext flash");
		}

		uint8_t stg_ble_key[EEP_BLE_SEC_KEY_LEN];
		memset(stg_ble_key, 0, sizeof(stg_ble_key));
		uint8_t key1_len = 0;
		ret = stg_config_blob_read(STG_BLOB_BLE_KEY, stg_ble_key, 
			&key1_len);
		if (ret != 0) {
			success = -1;
			LOG_ERR("STG Failed to host port");
		}
		LOG_INF("STG(BLE key): STG(%d) = 0x%X%X%X%X%X%X%X%X", key1_len, 
			stg_ble_key[0], stg_ble_key[1], stg_ble_key[2], 
			stg_ble_key[3], stg_ble_key[4], stg_ble_key[5], 
			stg_ble_key[6], stg_ble_key[7]);

		/* If every parameter was successfully copied from the eeprom
		 * to flash storage write 1 to the EEP_STG_COPY flag in the 
		 * EEPROM */
		if (success == 0) {
			ret = eep_uint8_write(EEP_STG_COPY, (uint8_t)1);
			if (ret != 0) {
				LOG_ERR("Failed to write to EEP_STG_COPY %i", 
					ret);
			}
		}
	}
#endif /* COPY_FROM_EEPROM */
	return ret;
}
#endif /* DT_NODE_HAS_STATUS(DT_ALIAS(eeprom), okay) */
