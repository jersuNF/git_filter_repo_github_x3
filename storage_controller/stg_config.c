#include "nclogs.h"

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
#include "nf_version.h"

LOG_MODULE_REGISTER(stg_config, CONFIG_STG_CONFIG_LOG_LEVEL);

/* Config parameter types */
enum {
	STG_INVALID_PARAM_TYPE = 0,
	STG_U8_PARAM_TYPE,
	STG_U16_PARAM_TYPE,
	STG_U32_PARAM_TYPE,
	STG_STR_PARAM_TYPE,
	STG_BLOB_PARAM_TYPE
} stg_param_type;

static const struct device *mp_device;
static const struct flash_area *mp_flash_area;
static struct nvs_fs m_file_system;
static bool m_initialized = false;

#ifdef CONFIG_STG_CONFIG_DEBUG_SEND_WRITE_ERRORS

uint16_t g_nvs_write_errors;
#define UPDATE_WRITE_ERRORS(rc)                                                                    \
	do {                                                                                       \
		if (rc < 0) {                                                                      \
			g_nvs_write_errors++;                                                      \
		}                                                                                  \
	} while (0)
#else
#define UPDATE_WRITE_ERRORS(rc)                                                                    \
	do {                                                                                       \
	} while (0)

#endif

#ifdef CONFIG_TEST
void stg_config_reset_for_unit_tests()
{
	mp_device = NULL;
	mp_flash_area = NULL;
	m_initialized = false;
}

struct nvs_fs *stg_config_get_file_system()
{
	return &m_file_system;
}

#endif

/* The following defines, give the NVS keys for FW 2004 production fields
 * needing to be migrated or moved
 */

#define STG_U32_DIAGNOSTIC_FLAGS_TODO_UNUSED_2004 25
#define STG_STR_MODEM_URAT_ARG_2004 26
#define STG_U16_ANO_START_ID_10002 26
#define STG_U16_ANO_ID_10002 27
#define STG_U32_ANO_TIMESTAMP_10002 28

/**
 * @brief Migrates NVS key/values on version change
 * @return 0 on success, negative on error
 */

static int do_migrations()
{
	char __aligned(4) buf[32];
	uint32_t version;

	/* Get the version we were at before rebooting to new */
	int rc = nvs_read(&m_file_system, (uint16_t)STG_U32_MIGRATED_VERSION, &version,
			  sizeof(version));
	if (rc == sizeof(version)) {
		if (version == NF_X25_VERSION_NUMBER) {
			/* We have already migrated to the version - we're done */
			return 0;
		}
	} else {
		NCLOG_WRN(STORAGE_CONTROLLER,TRice( iD( 4136), "wrn: Cannot get STG_U32_MIGRATED_VERSION. version == 0 and rc: %d\n", rc));
		/* We cannot deduce the version yet */
		version = 0;
	}
	/*
	 * If we cannot get the migrated version, we have to deduce if we were a
	 * 2023 FW 2004 production collar or a staging collar, typically 10002
	 */
	if (version == 0) {
		memset(buf, 0, sizeof(buf));
		/* String in 2004, U16 ANO-ID in 10004,  */
		rc = nvs_read(&m_file_system, (uint16_t)STG_STR_MODEM_URAT_ARG_2004, buf,
			      sizeof(buf));
		if (rc > 0) {
			if (rc == sizeof(uint16_t)) {
				/* some URAT string (9,7) or an ANO ID, if an ANO-ID, we
				 * know that STG_U32_ANO_TIMESTAMP_10002 is set as well */
				uint32_t ano_timestamp;
				int ret = nvs_read(&m_file_system,
						   (uint16_t)STG_U32_ANO_TIMESTAMP_10002,
						   &ano_timestamp, sizeof(ano_timestamp));
				if (ret != sizeof(ano_timestamp)) {
					version = 2004;
				} else {
					NCLOG_WRN(STORAGE_CONTROLLER,TRice( iD( 7931), "wrn: Failed to read STG_U32_ANO_TIMESTAMP_10002, default to version == 10002 and ret: %d\n", ret));
					version = 10002;
				}
			} else {
				version = 2004;
			}
		} else {
			NCLOG_WRN(STORAGE_CONTROLLER,TRice( iD( 2542), "wrn: Failed to read STG_STR_MODEM_URAT_ARG_2004,default to version == 2004 and rc: %d\n", rc));
			/* Cannot read ANO/URAT, default to URAT. String will be empty */
			version = 2004;
		}
	}
	/* Do the actual NVS migration */
	NCLOG_INF(STORAGE_CONTROLLER,TRice( iD( 6220),"inf: Migrating NVS from %d to %d\n", version, NF_X25_VERSION_NUMBER));
	if (version == 2004) {
		/* Must move the URAT to new location, write string + terminating \0 */
		rc = nvs_write(&m_file_system, STG_STR_MODEM_URAT_ARG, buf, strlen(buf) + 1);
		if (rc < 0) {
			NCLOG_ERR(STORAGE_CONTROLLER,TRice( iD( 1638),"err: Cannot write STG_STR_MODEM_URAT_ARG: %d\n", rc));
			return rc;
		}
	}
	/*
	 * Just leave any ANO parameters, as they are new non-existing keys
	 * That means that we just skip migration from 10002.
	 */

	/* Maintainer: Insert new checks here, noting that versions should go upwards only */
	version = NF_X25_VERSION_NUMBER;
	rc = nvs_write(&m_file_system, STG_U32_MIGRATED_VERSION, &version, sizeof(version));
	if (rc != sizeof(version)) {
		NCLOG_ERR(STORAGE_CONTROLLER,TRice( iD( 5765),"err: Cannot write STG_U32_MIGRATED_VERSION: %d\n", rc));
		return rc;
	}

	return 0;
}

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
			NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 1108),"err: STG Config, unable to open flash area with err: %d\n", err));
			return err;
		}

		NCLOG_DBG(STORAGE_CONTROLLER, TRice( iD( 2870),"dbg: STG Config: AreaID%d, FaID%d, FaOff%d, FaSize%d\n", flash_area_id, (uint8_t)mp_flash_area->fa_id, (int)mp_flash_area->fa_off, (int)mp_flash_area->fa_size));

		mp_device = device_get_binding(mp_flash_area->fa_dev_name);
		if (mp_device == NULL) {
			NCLOG_ERR(STORAGE_CONTROLLER, TRice0( iD( 3826),"err: STG Config, unable to get device\n"));
			return -ENODEV;
		}

		m_file_system.offset = (int)mp_flash_area->fa_off;
		m_file_system.sector_size = SECTOR_SIZE;
		m_file_system.sector_count = PM_CONFIG_PARTITION_SIZE / SECTOR_SIZE;

		err = nvs_init(&m_file_system, mp_device->name);
		if (err != 0) {
			NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 2868),"err: STG Config, failed to initialize NVS storage with err: %d\n", err));
			return err;
		}

		err = do_migrations();
		if (err != 0) {
			NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 3271),"err: STG Config, failed to migrate with err: %d\n", err));
			return err;
		}
		m_initialized = true;
	}
	return 0;
}

int stg_config_u8_read(stg_config_param_id_t id, uint8_t *value)
{
	int ret;
	if (m_initialized != true) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice0( iD( 3543),"wrn: STG Config not initialized\n"));
		return -ENODEV;
	}

	ret = is_valid_id(id);
	if (ret != STG_U8_PARAM_TYPE) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice( iD( 4111),"wrn: STG u8 read, invalid id %d, access denied with ret: %d\n", (int)id, ret));
		return -ENOMSG;
	}

	uint8_t val;
	ret = nvs_read(&m_file_system, (uint16_t)id, &val, sizeof(uint8_t));
	if (ret == -ENOENT) {
		/* Return u8 max if id-data pair does not exist */
		val = UINT8_MAX;
		NCLOG_WRN(STORAGE_CONTROLLER, TRice( iD( 6900), "wrn: STG u8 read, id-data pair does not exist at id %d\n", (int)id));
	} else if ((ret < 0) && (ret != -ENOENT)) {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 5382),"err: STG u8 read, failed to read storage at id %d with ret: %d \n", (int)id, ret));
		return ret;
	}
	*value = val;
	NCLOG_DBG(STORAGE_CONTROLLER, TRice( iD( 7360),"dbg: Read: Id=%d, Value=%d\n", (int)id, *value));
	return 0;
}

int stg_config_u8_write(stg_config_param_id_t id, const uint8_t value)
{
	int ret;
	if (m_initialized != true) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice0( iD( 5168),"wrn: STG Config not initialized\n"));
		return -ENODEV;
	}

	ret = is_valid_id(id);
	if (ret != STG_U8_PARAM_TYPE) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice( iD( 4546),"wrn: STG u8 write, invalid id %d, access denied with ret: %d\n", (int)id, ret));
		return -ENOMSG;
	}

	ret = nvs_write(&m_file_system, (uint16_t)id, &value, sizeof(uint8_t));
	if (ret < 0) {
		UPDATE_WRITE_ERRORS(ret);
		NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 5846),"err: STG u8 write, failed write to storage at id %d with ret: %d\n", (int)id, ret));
		return ret;
	}
	NCLOG_DBG(STORAGE_CONTROLLER, TRice( iD( 5313),"dbg: Write: Id=%d, Value=%d\n", (int)id, value));
	return 0;
}

int stg_config_u16_read(stg_config_param_id_t id, uint16_t *value)
{
	int ret;
	if (m_initialized != true) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice0( iD( 5555),"wrn: STG Config not initialized\n"));
		return -ENODEV;
	}

	ret = is_valid_id(id);
	if (ret != STG_U16_PARAM_TYPE) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice( iD( 3242),"wrn: STG u16 read, invalid id %d, access denied with ret: %d\n", (int)id, ret));
		return -ENOMSG;
	}

	uint16_t val;
	ret = nvs_read(&m_file_system, (uint16_t)id, &val, sizeof(uint16_t));
	if (ret == -ENOENT) {
		/* Return u16 max if id-data pair does not exist */
		val = UINT16_MAX;
		NCLOG_WRN(STORAGE_CONTROLLER, TRice( iD( 1184), "wrn: STG u16 read, id-data pair does not exist at id %d\n", (int)id));
	} else if ((ret < 0) && (ret != -ENOENT)) {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 7264),"err: STG u16 read, failed to read storage at id %d with ret: %d \n", (int)id, ret));
		return ret;
	}
	*value = val;
	NCLOG_DBG(STORAGE_CONTROLLER, TRice( iD( 1272),"dbg: Read: Id=%d, Value=%d\n", (int)id, *value));
	return 0;
}

int stg_config_u16_write(stg_config_param_id_t id, const uint16_t value)
{
	int ret;
	if (m_initialized != true) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice0( iD( 3333),"wrn: STG Config not initialized\n"));
		return -ENODEV;
	}

	ret = is_valid_id(id);
	if (ret != STG_U16_PARAM_TYPE) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice( iD( 7098),"wrn: STG u16 write, invalid id %d, access denied with ret: %d\n", (int)id, ret));
		return -ENOMSG;
	}

	ret = nvs_write(&m_file_system, (uint16_t)id, &value, sizeof(uint16_t));
	if (ret < 0) {
		UPDATE_WRITE_ERRORS(ret);
		NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 5199),"err: STG u16 write, failed write to storage at id %d with ret: %d\n", (int)id, ret));
		return ret;
	}
	NCLOG_DBG(STORAGE_CONTROLLER, TRice( iD( 4038),"dbg: Write: Id=%d, Value=%d\n", (int)id, value));
	return 0;
}

int stg_config_u32_read(stg_config_param_id_t id, uint32_t *value)
{
	int ret;
	if (m_initialized != true) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice0( iD( 6894),"wrn: STG Config not initialized\n"));
		return -ENODEV;
	}

	ret = is_valid_id(id);
	if (ret != STG_U32_PARAM_TYPE) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice( iD( 4154),"wrn: STG u32 read, invalid id %d, access denied with ret: %d\n", (int)id, ret));
		return -ENOMSG;
	}

	uint32_t val;
	ret = nvs_read(&m_file_system, (uint16_t)id, &val, sizeof(uint32_t));
	if (ret == -ENOENT) {
		/* Return u32 max if id-data pair does not exist */
		val = UINT32_MAX;
		NCLOG_WRN(STORAGE_CONTROLLER, TRice( iD( 3970), "wrn: STG u32 read, id-data pair does not exist at id %d\n", (int)id));
	} else if ((ret < 0) && (ret != -ENOENT)) {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 1209),"err: STG u32 read, failed to read storage at id %d with ret: %d \n", (int)id, ret));
		return ret;
	}
	*value = val;
	NCLOG_DBG(STORAGE_CONTROLLER, TRice( iD( 2082),"dbg: Read: Id=%d, Value=%d\n", (int)id, *value));
	return 0;
}

int stg_config_u32_write(stg_config_param_id_t id, const uint32_t value)
{
	int ret;
	if (m_initialized != true) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice0( iD( 7183),"wrn: STG Config not initialized\n"));
		return -ENODEV;
	}

	ret = is_valid_id(id);
	if (ret != STG_U32_PARAM_TYPE) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice( iD( 7902),"wrn: STG u32 write, invalid id %d, access denied with ret: %d\n", (int)id, ret));
		return -ENOMSG;
	}

	ret = nvs_write(&m_file_system, (uint16_t)id, &value, sizeof(uint32_t));
	if (ret < 0) {
		UPDATE_WRITE_ERRORS(ret);
		NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 5468),"err: STG u32 write, failed write to storage at id %d with ret: %d\n", (int)id, ret));
		return ret;
	}
	NCLOG_DBG(STORAGE_CONTROLLER, TRice( iD( 2275),"dbg: Write: Id=%d, Value=%d\n", (int)id, value));
	return 0;
}

int stg_config_str_read(stg_config_param_id_t id, char *buf, size_t bufsize, uint8_t *len)
{
	int ret;
	if (m_initialized != true) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice0( iD( 3827),"wrn: STG Config not initialized\n"));
		return -ENODEV;
	}

	ret = is_valid_id(id);
	if (ret != STG_STR_PARAM_TYPE) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice( iD( 2379), "wrn: invalid id (%d), access denied with ret: %d \n", (int)id, ret));
		return -ENOMSG;
	}

	memset(buf, 0, bufsize);
	ret = nvs_read(&m_file_system, (uint16_t)id, buf, bufsize);
	if (ret <= 0) {
		/* Return empty string if id-data pair does not exist or 0 buf size */
		NCLOG_WRN(STORAGE_CONTROLLER, TRice( iD( 2155), "wrn: STG string read, id-data pair does not exist at id %d with ret: %d \n", (int)id, ret));
		return ret;
	} else if (ret > bufsize) {
		/* The provided buffer was too small */
		return -ERANGE;
	} else if (ret == bufsize) {
		/* If string is not null-terminated, we are unititialized  */
		if (buf[ret - 1] != '\0') {
			return -ENODATA;
		}
	}
	*len = strlen(buf);
	NCLOG_DBG(STORAGE_CONTROLLER, TRice( iD( 6970),"dbg: Read: Id=%d, Length=%d, Data=dynamic_string\n", (int)id, *len));
	return 0;
}

int stg_config_str_write(stg_config_param_id_t id, const char *str, size_t len)
{
	int ret;
	if (m_initialized != true) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice0( iD( 7057),"wrn: STG Config not initialized\n"));
		return -ENODEV;
	}

	ret = is_valid_id(id);
	if (ret != STG_STR_PARAM_TYPE) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice( iD( 7656),"wrn: STG str write, invalid id %d, access denied with ret: %d\n", (int)id, ret));
		return -ENOMSG;
	}
	/* TODO, make this much more elegant with compile-time checks :-( */
	size_t max_size;
	if (id == STG_STR_HOST_PORT) {
		max_size = STG_CONFIG_HOST_PORT_BUF_LEN;
	} else if (id == STG_STR_MODEM_URAT_ARG) {
		max_size = STG_CONFIG_URAT_ARG_BUF_SIZE;
	} else {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 2265),"err: Invalid string key: %d\n", (int)id));
		return -EINVAL;
	}
	if (len + 1 > max_size) {
		return -EOVERFLOW;
	}

	/* Write the string including the terminating zero to NVS */
	ret = nvs_write(&m_file_system, (uint16_t)id, str, len + 1);
	if (ret < 0) {
		UPDATE_WRITE_ERRORS(ret);
		NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 5384),"err: STG str write, failed write to storage at id %d with ret: %d\n", (int)id, ret));
		return ret;
	}
	NCLOG_DBG(STORAGE_CONTROLLER, TRice( iD( 7692),"dbg: Read: Id=%d, Length=%d, Data=dynamic_string\n", (int)id, strlen(str), str));
	return 0;
}

int stg_config_blob_read(stg_config_param_id_t id, uint8_t *arr, uint8_t *len)
{
	int ret;
	if (m_initialized != true) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice0( iD( 4341),"wrn: STG Config not initialized\n"));
		return -ENODEV;
	}

	ret = is_valid_id(id);
	if (ret != STG_BLOB_PARAM_TYPE) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice( iD( 6733),"wrn: STG blob read, invalid id %d, access denied with ret: %d\n", (int)id, ret));
		return -ENOMSG;
	}

	uint8_t buff[(id == STG_BLOB_BLE_KEY) ? STG_CONFIG_BLE_SEC_KEY_LEN : 0];
	if (sizeof(buff) <= 0) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice( iD( 6476),"wrn: STG blob read, unknown data size %d\n", sizeof(buff)));
		return -ENOMSG;
	}
	memset(buff, 0, sizeof(buff));

	ret = nvs_read(&m_file_system, (uint16_t)id, &buff, sizeof(buff));
	if (ret == -ENOENT) {
		/* Return nothing if blob id-data pair does not exist */
		NCLOG_WRN(STORAGE_CONTROLLER, TRice( iD( 3602),"wrn: STG blob read, id-data pair does not exist at id %d\n", (int)id));
		return 0;
	} else if ((ret < 0) && (ret != -ENOENT)) {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 3561),"err: STG blob read, failed read to storage at id %d with ret: %d \n", (int)id, ret));
		return ret;
	}
	memcpy(arr, buff, sizeof(buff));
	*len = sizeof(buff);

	NCLOG_DBG(STORAGE_CONTROLLER, TRice( iD( 3482),"dbg: Read: Id=%d, Length=%d, Data=0x%X%X%X%X%X%X%X%X\n", (int)id, *len, arr[0], arr[1], arr[2], arr[3], arr[4], arr[5], arr[6], arr[7]));

	return 0;
}

int stg_config_blob_write(stg_config_param_id_t id, const uint8_t *arr, const uint8_t len)
{
	int ret;
	if (m_initialized != true) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice0( iD( 5502),"wrn: STG Config not initialized\n"));
		return -ENODEV;
	}

	ret = is_valid_id(id);
	if (ret != STG_BLOB_PARAM_TYPE) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice( iD( 7314),"wrn: STG blob write, invalid id %d, access denied with ret: %d\n", (int)id, ret));
		return -ENOMSG;
	}
	if ((id == STG_BLOB_BLE_KEY) && (len > STG_CONFIG_BLE_SEC_KEY_LEN)) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice( iD( 2878),"wrn: STG blb write, incorrect size %d for id %d\n", len, (int)id));
		return -EOVERFLOW;
	}

	ret = nvs_write(&m_file_system, (uint16_t)id, arr, len);
	if (ret < 0) {
		UPDATE_WRITE_ERRORS(ret);
		NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 6463),"err: STG blob write, failed write to storage at id %d with ret: %d\n", (int)id, ret));
		return ret;
	}

	NCLOG_DBG(STORAGE_CONTROLLER, TRice( iD( 4882),"dbg: Read: Id=%d, Length=%d, Data=0x%X%X%X%X%X%X%X%X\n", (int)id, len, arr[0], arr[1], arr[2], arr[3], arr[4], arr[5], arr[6], arr[7]));

	return 0;
}

int stg_config_erase_all()
{
	int ret;
	if (m_initialized != true) {
		NCLOG_WRN(STORAGE_CONTROLLER, TRice0( iD( 4137),"wrn: STG Config not initialized\n"));
		return -ENODEV;
	}

	ret = flash_area_erase(mp_flash_area, (uint32_t)mp_flash_area->fa_off,
			       (uint32_t)mp_flash_area->fa_size);
	if (ret != 0) {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 3971),"err: STG Config, unable to erase flash sectors, err %d\n", ret));
		return ret;
	}
	return 0;
}

/* TODO PSH fixme, make this compile time checks instead */
int is_valid_id(stg_config_param_id_t param_id)
{
	int param_type;
	switch ((stg_config_param_id_t)param_id) {
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
	case STG_U8_M_UPLOAD_PERIODIC_LOGS:
	case STG_U8_M_UPLOAD_CORE_DUMPS:
	case STG_U8_MODEM_INSTALLING: {
		param_type = STG_U8_PARAM_TYPE;
		break;
	}
	case STG_U16_ACC_SIGMA_NOACTIVITY_LIMIT:
	case STG_U16_OFF_ANIMAL_TIME_LIMIT_SEC:
	case STG_U16_ACC_SIGMA_SLEEP_LIMIT:
	case STG_U16_ZAP_CNT_TOT:
	case STG_U16_ZAP_CNT_DAY:
	case STG_U16_PRODUCT_TYPE:
	case STG_U16_ANO_ID:
	case STG_U16_ANO_START_ID:
	case STG_U16_LAST_GOOD_ANO_ID: {
		param_type = STG_U16_PARAM_TYPE;
		break;
	}
	case STG_U32_UID:
	case STG_U32_ANO_TIMESTAMP:
	case STG_U32_WARN_CNT_TOT: {
		param_type = STG_U32_PARAM_TYPE;
		break;
	}
	case STG_STR_HOST_PORT:
	case STG_STR_MODEM_URAT_ARG: {
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
		NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 6578),"err: EEP(u8) Failed to read id %d, err %d\n", EEP_STG_COPY, ret));
		return ret;
	}

	uint32_t eep_serial_no = 0;
	ret = eep_uint32_read(EEP_UID, &eep_serial_no);
	if (ret != 0) {
		NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 2714),"err: EEP(u8) Failed to read id %d, err %d\n", EEP_UID, ret));
		return ret;
	}

	/* Do a compare and copy if EEP_STG_COPY flag is 0 */
	int success = 0;
	if (((eep_stg_copy_val == 0) || (eep_stg_copy_val == UINT8_MAX)) && (eep_serial_no > 0) &&
	    (eep_serial_no != UINT32_MAX)) {
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
				NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 4204),"err: EEP(u8) Failed to read id %d, err %d\n", i, ret));
			}

			/* Read uint8 STG value */
			stg_u8_val = 0;
			ret = stg_config_u8_read(i, &stg_u8_val);
			if (ret != 0) {
				success = -1;
				NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 5845),"err: STG(u8) Failed to read id %d, err %d\n", i, ret));
			}
			NCLOG_INF(STORAGE_CONTROLLER, TRice( iD( 5162),"inf: EEPvsSTG(u8): Id = %d, EEP = %d, STG = %d\n", i, eep_u8_val, stg_u8_val));

			/* Compare and copy if eeprom and stg flash values are
			 * not identical */
			if (eep_u8_val != stg_u8_val) {
				ret = stg_config_u8_write(i, eep_u8_val);
				if (ret != 0) {
					success = -1;
					NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 3684),"err: STG(u8) Failed to write id %d, err %d\n", i, ret));
				}

				stg_u8_val = 0;
				ret = stg_config_u8_read(i, &stg_u8_val);
				if (ret != 0) {
					success = -1;
					NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 6270),"err: STG(u8) Failed to read id %d, err %d\n", i, ret));
				}
				NCLOG_INF(STORAGE_CONTROLLER, TRice( iD( 2422),"inf: EEPvsSTG(u8): Id = %d, EEP = %d, STG = %d\n", i, eep_u8_val, stg_u8_val));
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
				NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 5451),"err: EEP(u16) Failed to read id %d, err %d\n", i, ret));
			}

			/* Read uint16 STG value */
			stg_u16_val = 0;
			ret = stg_config_u16_read(i + 14, &stg_u16_val);
			if (ret != 0) {
				success = -1;
				NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 7563),"err: STG(u16) Failed to read id %d(+14), err %d\n", i, ret));
			}
			NCLOG_INF(STORAGE_CONTROLLER, TRice( iD( 6625),"inf: EEPvsSTG(u16): Id = %d(+14), EEP = %d, STG = %d\n", i, eep_u16_val, stg_u16_val));

			/* Compare and copy if eeprom and stg flash values are
			 * not identical */
			if (eep_u16_val != stg_u16_val) {
				ret = stg_config_u16_write(i + 14, eep_u16_val);
				if (ret != 0) {
					success = -1;
					NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 7967),"err: STG(u16) Failed to write id %d(+14), err %d\n", i, ret));
				}

				stg_u16_val = 0;
				ret = stg_config_u16_read(i + 14, &stg_u16_val);
				if (ret != 0) {
					success = -1;
					NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 6045),"err: STG(u16) Failed to read id %d(+14), err %d\n", i, ret));
				}
				NCLOG_INF(STORAGE_CONTROLLER, TRice( iD( 1155),"inf: EEPvsSTG(u16): Id = %d(+14), EEP = %d, STG = %d\n", i, eep_u16_val, stg_u16_val));
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
				NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 5772),"err: EEP(u32) Failed to read id %d, err %d\n", i, ret));
			}

			/* Read uint32 STG value */
			stg_u32_val = 0;
			ret = stg_config_u32_read(i + 20, &stg_u32_val);
			if (ret != 0) {
				success = -1;
				NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 7989),"err: STG(u32) Failed to read id %d(+20), err %d\n", i, ret));
			}
			NCLOG_INF(STORAGE_CONTROLLER, TRice( iD( 7750),"inf: EEPvsSTG(u32): Id = %d(+20), EEP = %d, STG = %d\n", i, eep_u32_val, stg_u32_val));

			/* Compare and copy if eeprom and stg flash values are
			 * not identical */
			if (eep_u32_val != stg_u32_val) {
				ret = stg_config_u32_write(i + 20, eep_u32_val);
				if (ret != 0) {
					success = -1;
					NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 7046),"err: STG(u32) Failed to write id %d(+20), err %d\n", i, ret));
				}

				stg_u32_val = 0;
				ret = stg_config_u32_read(i + 20, &stg_u32_val);
				if (ret != 0) {
					success = -1;
					NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 3547),"err: STG(u32) Failed to read id %d(+20), err %d\n", i, ret));
				}
				NCLOG_INF(STORAGE_CONTROLLER, TRice( iD( 4147),"inf: EEPvsSTG(u32): Id = %d(+20), EEP = %d, STG = %d\n", i, eep_u32_val, stg_u32_val));
			}
		}

		/* Copy Host Port from the EEPROM to the flash */
		char port[STG_CONFIG_HOST_PORT_BUF_LEN];
		memset(port, 0, sizeof(port));

		/* Read Host Port from EEPROM */
		ret = eep_read_host_port(&port[0], STG_CONFIG_HOST_PORT_BUF_LEN);
		if (ret != 0) {
			success = -1;
			NCLOG_ERR(STORAGE_CONTROLLER, TRice0( iD( 1156),"err: EEP Failed to read host port\n"));
		}
		NCLOG_INF(STORAGE_CONTROLLER, TRice0( iD( 4156),"inf: EEPvsSTG(Host port): EEP = dynamic_string\n"));

		ret = stg_config_str_write(STG_STR_HOST_PORT, port, STG_CONFIG_HOST_PORT_BUF_LEN);
		if (ret != 0) {
			success = -1;
			NCLOG_ERR(STORAGE_CONTROLLER, TRice0( iD( 1573),"err: STG Failed to write host port to ext flash!\n"));
		}

		char port_read[STG_CONFIG_HOST_PORT_BUF_LEN];
		uint8_t port_len = 0;
		ret = stg_config_str_read(STG_STR_HOST_PORT, port_read, sizeof(port_read),
					  &port_len);
		if (ret != 0) {
			success = -1;
			NCLOG_ERR(STORAGE_CONTROLLER, TRice0( iD( 6594),"err: STG Failed to read host port\n"));
		}
		NCLOG_INF(STORAGE_CONTROLLER, TRice( iD( 6174),"inf: EEPvsSTG(Host port): STG%d = dynamic_string\n", port_len, port_read));

		/* Copy BLE key from the EEPROM to the flash */
		uint8_t ble_key[EEP_BLE_SEC_KEY_LEN];
		memset(ble_key, 0, sizeof(ble_key));

		/* Read BLE Security key from EEPROM */
		ret = eep_read_ble_sec_key(ble_key, EEP_BLE_SEC_KEY_LEN);
		if (ret != 0) {
			success = -1;
			NCLOG_ERR(STORAGE_CONTROLLER, TRice0( iD( 3253),"err: EEP Failed to read ble key from EEPROM\n"));
		}
		NCLOG_INF(STORAGE_CONTROLLER, TRice( iD( 5568),"inf: EEP(BLE key): EEP = 0x%X%X%X%X%X%X%X%X\n", ble_key[0], ble_key[1], ble_key[2], ble_key[3], ble_key[4], ble_key[5], ble_key[6], ble_key[7]));

		/* Write BLE Security key to flash */
		ret = stg_config_blob_write(STG_BLOB_BLE_KEY, ble_key, EEP_BLE_SEC_KEY_LEN);
		if (ret != 0) {
			success = -1;
			NCLOG_ERR(STORAGE_CONTROLLER, TRice0( iD( 2003),"err: STG Failed to write ble key to ext flash\n"));
		}

		uint8_t stg_ble_key[EEP_BLE_SEC_KEY_LEN];
		memset(stg_ble_key, 0, sizeof(stg_ble_key));
		uint8_t key1_len = 0;
		ret = stg_config_blob_read(STG_BLOB_BLE_KEY, stg_ble_key, &key1_len);
		if (ret != 0) {
			success = -1;
			NCLOG_ERR(STORAGE_CONTROLLER, TRice0( iD( 4900),"err: STG Failed to host port\n"));
		}
		NCLOG_INF(STORAGE_CONTROLLER, TRice( iD( 1510),"inf: STG(BLE key): STG%d = 0x%X%X%X%X%X%X%X%X\n", key1_len, stg_ble_key[0], stg_ble_key[1], stg_ble_key[2], stg_ble_key[3], stg_ble_key[4], stg_ble_key[5], stg_ble_key[6], stg_ble_key[7]));

		/* If every parameter was successfully copied from the eeprom
		 * to flash storage write 1 to the EEP_STG_COPY flag in the 
		 * EEPROM */
		if (success == 0) {
			ret = eep_uint8_write(EEP_STG_COPY, (uint8_t)1);
			if (ret != 0) {
				NCLOG_ERR(STORAGE_CONTROLLER, TRice( iD( 3562),"err: Failed to write to EEP_STG_COPY %i\n", ret));
			}
		}
	}
#endif /* COPY_FROM_EEPROM */
	return ret;
}
#endif /* DT_NODE_HAS_STATUS(DT_ALIAS(eeprom), okay) */
