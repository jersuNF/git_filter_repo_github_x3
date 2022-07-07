
/*
* Copyright (c) 2022 Nofence AS
*/

#include "nf_settings_private.h"
#include <drivers/eeprom.h>
#include <string.h>
#include "amc_const.h"
#include "embedded.pb.h"
#include <logging/log.h>
#include "nf_settings.h"
#include "movement_controller.h"

LOG_MODULE_REGISTER(nf_settings, 3);

#define EEPROM_DEFAULT_VALUE_8_T 0xFF
#define EEPROM_DEFAULT_VALUE_16_T 0xFFFF
#define EEPROM_DEFAULT_VALUE_32_T 0xFFFFFFFF

/* EEPROM device pointer */
const struct device *m_p_device;

void eep_init(const struct device *dev)
{
	m_p_device = dev;
}

int eep_write_host_port(const char *host_port)
{
	if (strlen(host_port) > EEP_HOST_PORT_BUF_SIZE - 1) {
		return -EOVERFLOW;
	}
	/* Note, write the string including null-terminator */
	return eeprom_write(m_p_device, offsetof(struct eemem, eep_host_port),
			    host_port, strlen(host_port) + 1);
}

int eep_read_host_port(char *host_port, size_t bufsize)
{
	if (bufsize < EEP_HOST_PORT_BUF_SIZE) {
		return -EOVERFLOW;
	}
	int ret = eeprom_read(m_p_device, offsetof(struct eemem, eep_host_port),
			      host_port, EEP_HOST_PORT_BUF_SIZE);
	if (ret == 0) {
		host_port[EEP_HOST_PORT_BUF_SIZE - 1] = '\0';
	}
	return ret;
}

/** @todo Create macros instead to directly convert enum to offset. */
int eep_uint8_read(eep_uint8_enum_t field, uint8_t *value)
{
	off_t offset;
	int ret = 0;

	switch (field) {
	case EEP_WARN_MAX_DURATION: {
		offset = offsetof(struct eemem, eep_warn_max_duration);

		/* Check for default values, i.e value read is 0xFF,
		 * set to defualt WARN_MAX value. 
		 */
		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
		*value = *value == EEPROM_DEFAULT_VALUE_8_T ?
				 WARN_MAX_DURATION :
				 *value;
		LOG_INF("Read EEP_WARN_MAX_DURATION %i", *value);
		break;
	}
	case EEP_WARN_MIN_DURATION: {
		offset = offsetof(struct eemem, eep_warn_min_duration);

		/* Check for default values, i.e value read is 0xFF,
		 * set to defualt WARN_MIN value. 
		 */
		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
		*value = *value == EEPROM_DEFAULT_VALUE_8_T ?
				 WARN_MIN_DURATION :
				 *value;
		LOG_INF("Read EEP_WARN_MIN_DURATION %i", *value);
		break;
	}
	case EEP_PAIN_CNT_DEF_ESCAPED: {
		offset = offsetof(struct eemem, eep_warn_min_duration);

		/* Check for default values, i.e value read is 0xFF,
		 * set to default CNT_DEF_ESCAPED value. 
		 */
		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
		*value = *value == EEPROM_DEFAULT_VALUE_8_T ?
				 PAIN_CNT_DEF_ESCAPED :
				 *value;
		LOG_INF("Read EEP_PAIN_CNT_DEF_ESCAPED %i", *value);
		break;
	}
	case EEP_FENCE_STATUS: {
		offset = offsetof(struct eemem, eep_fence_status);

		/* Check for default values, i.e value read is 0xFF,
		 * set to defualt FenceStatus_FenceStatus_UNKNOWN. 
		 */
		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
		*value = *value == EEPROM_DEFAULT_VALUE_8_T ?
				 FenceStatus_FenceStatus_UNKNOWN :
				 *value;
		LOG_INF("Read EEP_FENCE_STATUS %i", *value);
		break;
	}
	case EEP_COLLAR_MODE: {
		offset = offsetof(struct eemem, eep_collar_mode);

		/* Check for default values, i.e value read is 0xFF,
		 * set to defualt Mode_Mode_UNKNOWN. 
		 */
		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
		*value = *value == EEPROM_DEFAULT_VALUE_8_T ?
				 Mode_Mode_UNKNOWN :
				 *value;
		LOG_INF("Read EEP_COLLAR_MODE %i", *value);
		break;
	}
	case EEP_COLLAR_STATUS: {
		offset = offsetof(struct eemem, eep_collar_status);

		/* Check for default values, i.e value read is 0xFF,
		 * set to defualt CollarStatus_CollarStatus_UNKNOWN. 
		 */
		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
		*value = *value == EEPROM_DEFAULT_VALUE_8_T ?
				 CollarStatus_CollarStatus_UNKNOWN :
				 *value;
		LOG_INF("Read EEP_COLLAR_STATUS %i", *value);
		break;
	}
	case EEP_TEACH_MODE_FINISHED: {
		offset = offsetof(struct eemem, eep_teach_mode_finished);

		/* Check for default values, i.e value read is 0xFF,
		 * set to defualt 0, not finished. 
		 */
		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
		*value = *value == EEPROM_DEFAULT_VALUE_8_T ? 0 : *value;
		LOG_INF("Read teach mode finished %i", *value);
		break;
	}
	case EEP_KEEP_MODE: {
		offset = offsetof(struct eemem, eep_keep_mode);
		/* Check for default values, i.e value read is 0xFF,
		 * set to defualt 0, not finished. 
		 */
		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
		*value = *value == EEPROM_DEFAULT_VALUE_8_T ? 0 : *value;
		LOG_INF("Read keep mode %i", *value);
		break;
	}
	case EEP_EMS_PROVIDER: {
		offset = offsetof(struct eemem, eep_ems_provider);

		/* Check for default values, i.e value read is 0xFF,
		 * set to defualt 0, not finished. 
		 */
		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
		*value = *value == EEPROM_DEFAULT_VALUE_8_T ? 0 : *value;
		LOG_INF("Set EMS provider to %i", *value);
		break;
	}
	case EEP_PRODUCT_RECORD_REV: {
		offset = offsetof(struct eemem, eep_product_record_rev);

		/* Check for default values, i.e value read is 0xFF,
		 * set to defualt 0, not finished. 
		 */
		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
		*value = *value == EEPROM_DEFAULT_VALUE_8_T ? 0 : *value;
		LOG_INF("Set product record rev to %i", *value);
		break;
	}
	case EEP_BOM_MEC_REV: {
		offset = offsetof(struct eemem, eep_bom_mec_rev);

		/* Check for default values, i.e value read is 0xFF,
		 * set to defualt 0, not finished. 
		 */
		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
		*value = *value == EEPROM_DEFAULT_VALUE_8_T ? 0 : *value;
		LOG_INF("Set BOM mec rev to %i", *value);
		break;
	}
	case EEP_BOM_PCB_REV: {
		offset = offsetof(struct eemem, eep_bom_pcb_rev);

		/* Check for default values, i.e value read is 0xFF,
		 * set to defualt 0, not finished. 
		 */
		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
		*value = *value == EEPROM_DEFAULT_VALUE_8_T ? 0 : *value;
		LOG_INF("Set BOM pcb rev to %i", *value);
		break;
	}
	case EEP_HW_VERSION: {
		offset = offsetof(struct eemem, eep_hw_version);

		/* Check for default values, i.e value read is 0xFF,
		 * set to defualt 0, not finished. 
		 */
		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
		*value = *value == EEPROM_DEFAULT_VALUE_8_T ? 0 : *value;
		LOG_INF("Set hw version to %i", *value);
		break;
	}
	default: {
		return -EINVAL;
	}
	}

	return ret;
}

int eep_uint8_write(eep_uint8_enum_t field, uint8_t value)
{
	off_t offset;

	switch (field) {
	case EEP_WARN_MAX_DURATION: {
		offset = offsetof(struct eemem, eep_warn_max_duration);
		break;
	}
	case EEP_WARN_MIN_DURATION: {
		offset = offsetof(struct eemem, eep_warn_min_duration);
		break;
	}
	case EEP_PAIN_CNT_DEF_ESCAPED: {
		offset = offsetof(struct eemem, eep_pain_cnt_def_escaped);
		break;
	}
	case EEP_COLLAR_MODE: {
		offset = offsetof(struct eemem, eep_collar_mode);
		break;
	}
	case EEP_FENCE_STATUS: {
		offset = offsetof(struct eemem, eep_fence_status);
		break;
	}
	case EEP_COLLAR_STATUS: {
		offset = offsetof(struct eemem, eep_collar_status);
		break;
	}
	case EEP_TEACH_MODE_FINISHED: {
		offset = offsetof(struct eemem, eep_teach_mode_finished);
		break;
	}
	case EEP_EMS_PROVIDER: {
		offset = offsetof(struct eemem, eep_ems_provider);
		break;
	}
	case EEP_PRODUCT_RECORD_REV: {
		offset = offsetof(struct eemem, eep_product_record_rev);
		break;
	}
	case EEP_BOM_MEC_REV: {
		offset = offsetof(struct eemem, eep_bom_mec_rev);
		break;
	}
	case EEP_BOM_PCB_REV: {
		offset = offsetof(struct eemem, eep_bom_pcb_rev);
		break;
	}
	case EEP_HW_VERSION: {
		offset = offsetof(struct eemem, eep_hw_version);
		break;
	}
	case EEP_KEEP_MODE: {
		offset = offsetof(struct eemem, eep_keep_mode);
		break;
	}
	default: {
		return -EINVAL;
	}
	}

	return eeprom_write(m_p_device, offset, &value, sizeof(value));
}

int eep_uint16_read(eep_uint16_enum_t field, uint16_t *value)
{
	off_t offset;
	int ret = 0;

	switch (field) {
	case EEP_ACC_SIGMA_NOACTIVITY_LIMIT: {
		offset = offsetof(struct eemem, eep_acc_sigma_noactivity_limit);

		/* Check for default values, i.e value read is 0xFFFF,
		 * set to defualt 0 total zaps. 
		 */
		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
		*value = *value == EEPROM_DEFAULT_VALUE_16_T ?
				 ACC_SIGMA_NOACTIVITY_LIMIT_DEFAULT :
				 *value;
		LOG_INF("Set EEP_ACC_SIGMA_NOACTIVITY_LIMIT to %i", *value);
		break;
	}
	case EEP_OFF_ANIMAL_TIME_LIMIT_SEC: {
		offset = offsetof(struct eemem, eep_off_animal_time_limit);

		/* Check for default values, i.e value read is 0xFFFF,
		 * set to defualt 0 total zaps. 
		 */
		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
		*value = *value == EEPROM_DEFAULT_VALUE_16_T ?
				 OFF_ANIMAL_TIME_LIMIT_SEC_DEFAULT :
				 *value;
		LOG_INF("Set EEP_OFF_ANIMAL_TIME_LIMIT_SEC to %i", *value);
		break;
	}
	case EEP_ACC_SIGMA_SLEEP_LIMIT: {
		offset = offsetof(struct eemem, eep_acc_sigma_sleep_limit);

		/* Check for default values, i.e value read is 0xFFFF,
		 * set to defualt 0 total zaps. 
		 */
		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
		*value = *value == EEPROM_DEFAULT_VALUE_16_T ?
				 ACC_SIGMA_SLEEP_LIMIT_DEFAULT :
				 *value;
		LOG_INF("Set EEP_ACC_SIGMA_SLEEP_LIMIT to %i", *value);
		break;
	}
	case EEP_ZAP_CNT_TOT: {
		offset = offsetof(struct eemem, eep_zap_cnt_tot);

		/* Check for default values, i.e value read is 0xFFFF,
		 * set to defualt 0 total zaps. 
		 */
		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
		*value = *value == EEPROM_DEFAULT_VALUE_16_T ? 0 : *value;
		LOG_INF("Set EEP_ZAP_CNT_TOT to %i", *value);
		break;
	}
	case EEP_ZAP_CNT_DAY: {
		offset = offsetof(struct eemem, eep_zap_cnt_day);

		/* Check for default values, i.e value read is 0xFFFF,
		 * set to defualt 0 total zaps. 
		 */
		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
		*value = *value == EEPROM_DEFAULT_VALUE_16_T ? 0 : *value;
		LOG_INF("Set EEP_ZAP_CNT_DAY to %i", *value);
		break;
	}
	case EEP_PRODUCT_TYPE: {
		offset = offsetof(struct eemem, eep_product_type);

		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
		LOG_INF("Set EEP_PRODUCT_TYPE to %i", *value);
		break;
	}
	default: {
		return -EINVAL;
	}
	}

	return ret;
}

int eep_uint16_write(eep_uint16_enum_t field, uint16_t value)
{
	off_t offset;

	switch (field) {
	case EEP_ACC_SIGMA_NOACTIVITY_LIMIT: {
		offset = offsetof(struct eemem, eep_acc_sigma_noactivity_limit);
		break;
	}
	case EEP_OFF_ANIMAL_TIME_LIMIT_SEC: {
		offset = offsetof(struct eemem, eep_off_animal_time_limit);
		break;
	}
	case EEP_ACC_SIGMA_SLEEP_LIMIT: {
		offset = offsetof(struct eemem, eep_acc_sigma_sleep_limit);
		break;
	}
	case EEP_ZAP_CNT_TOT: {
		offset = offsetof(struct eemem, eep_zap_cnt_tot);
		break;
	}
	case EEP_ZAP_CNT_DAY: {
		offset = offsetof(struct eemem, eep_zap_cnt_day);
		break;
	}
	case EEP_PRODUCT_TYPE: {
		offset = offsetof(struct eemem, eep_product_type);
		break;
	}
	default: {
		return -EINVAL;
	}
	}

	return eeprom_write(m_p_device, offset, &value, sizeof(value));
}

int eep_uint32_read(eep_uint32_enum_t field, uint32_t *value)
{
	off_t offset;
	int ret = 0;

	switch (field) {
	case EEP_UID: {
		offset = offsetof(struct eemem, eep_uid);

		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
		/** @todo Any default IDs to fetch???? What do to? */
		//*value = *value == EEPROM_DEFAULT_VALUE_32_T ? 0 : *value;
		//LOG_INF("Set EEP_UID to %i", *value);
		break;
	}
	case EEP_WARN_CNT_TOT: {
		offset = offsetof(struct eemem, eep_warn_cnt_tot);

		/* Check for default values, i.e value read is 0xFFFFFFFF,
		 * set to defualt 0 total warnings. 
		 */
		ret = eeprom_read(m_p_device, offset, value, sizeof(*value));
		*value = *value == EEPROM_DEFAULT_VALUE_32_T ? 0 : *value;
		LOG_INF("Set EEP_WARN_CNT_TOT to %i", *value);
		break;
	}
	default: {
		return -EINVAL;
	}
	}

	return ret;
}

int eep_uint32_write(eep_uint32_enum_t field, uint32_t value)
{
	off_t offset;

	switch (field) {
	case EEP_UID: {
		offset = offsetof(struct eemem, eep_uid);
		break;
	}
	case EEP_WARN_CNT_TOT: {
		offset = offsetof(struct eemem, eep_warn_cnt_tot);
		break;
	}
	default: {
		return -EINVAL;
	}
	}

	return eeprom_write(m_p_device, offset, &value, sizeof(value));
}

int eep_write_ble_sec_key(uint8_t *ble_sec_key, size_t bufsize)
{
	if (sizeof(bufsize) > EEP_BLE_SEC_KEY_LEN) {
		return -EOVERFLOW;
	}
	/* Note, write the string including null-terminator */
	return eeprom_write(m_p_device, offsetof(struct eemem, ble_sec_key),
			    ble_sec_key, bufsize);
}

int eep_read_ble_sec_key(uint8_t *ble_sec_key, size_t bufsize)
{
	if (bufsize < EEP_BLE_SEC_KEY_LEN) {
		return -EOVERFLOW;
	}
	int ret = eeprom_read(m_p_device, offsetof(struct eemem, ble_sec_key),
			      ble_sec_key, EEP_BLE_SEC_KEY_LEN);
	return ret;
}
