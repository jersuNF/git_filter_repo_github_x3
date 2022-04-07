
/*
* Copyright (c) 2022 Nofence AS
*/

#include "nf_settings_private.h"
#include <drivers/eeprom.h>
#include <string.h>

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

	switch (field) {
	case EEP_WARN_MAX_DURATION: {
		offset = offsetof(struct eemem, eep_warn_max_duration);
		break;
	}
	case EEP_OFF_ANIMAL_TIME_LIMIT_SEC: {
		offset = offsetof(struct eemem, eep_warn_min_duration);
		break;
	}
	case EEP_ACC_SIGMA_SLEEP_LIMIT: {
		offset = offsetof(struct eemem, eep_pain_cnt_def_escaped);
		break;
	}
	case EEP_ZAP_CNT_TOT: {
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
	default: {
		return -EINVAL;
	}
	}

	return eeprom_read(m_p_device, offset, value, sizeof(*value));
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
	default: {
		return -EINVAL;
	}
	}

	return eeprom_write(m_p_device, offset, &value, sizeof(value));
}

int eep_uint16_read(eep_uint16_enum_t field, uint16_t *value)
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
	default: {
		return -EINVAL;
	}
	}

	return eeprom_read(m_p_device, offset, value, sizeof(*value));
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
	default: {
		return -EINVAL;
	}
	}

	return eeprom_write(m_p_device, offset, &value, sizeof(value));
}

int eep_uint32_read(eep_uint32_enum_t field, uint32_t *value)
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

	return eeprom_read(m_p_device, offset, value, sizeof(*value));
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