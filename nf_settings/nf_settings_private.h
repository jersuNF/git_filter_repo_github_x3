
/*
* Copyright (c) 2022 Nofence AS
*/

#ifndef X3_FW_NF_SETTINGS_PRIVATE_H
#define X3_FW_NF_SETTINGS_PRIVATE_H

#include <stdint.h>
#include <nf_settings.h>

/**
 * @brief Convenience structure to define the Zephyr eeprom address space
 * for storing/retrieving values
 *
 * Typically, we use the C library macro @p offsetof to determine
 * the address to use with Zephyr @p eeprom_read and @p eeprom_write
 *
 * @note You are not meant to instantiate this structure outside the Nofence
 * eeprom API.
 */
__packed struct eemem {
	/** The collar serial number, must always be the first member */
	uint32_t eep_uid;

	/** Configurable 0-terminated host:port, e.g. @p 192.176.777.888:987654\0 */
	char eep_host_port[EEP_HOST_PORT_BUF_SIZE];

	/** Used to determine no activity from the accelerometer. */
	uint16_t eep_acc_sigma_noactivity_limit;

	/** Used to declare collar as "OffAnimal" after x seconds of inactivity. */
	uint16_t eep_off_animal_time_limit;

	/** From 147.x.x.x, possible to set overall accelormeter Std sleep limit. */
	uint16_t eep_acc_sigma_sleep_limit;

	/** Total number of zaps during  the lifetime of the collar. */
	uint16_t eep_zap_cnt_tot;

	/** Counter of zaps per 24 hours. */
	uint16_t eep_zap_cnt_day;

	/** Total number of warning signals. 
	  * FIXME : Only write periodically from RAM value. 
	  */
	uint32_t eep_warn_cnt_tot;

	/** Max seconds for warnings. */
	uint8_t eep_warn_max_duration;

	/** Min seconds for warnings. */
	uint8_t eep_warn_min_duration;

	/** Number of zaps before animal is considered as escaped. */
	uint8_t eep_pain_cnt_def_escaped;

	/** Current Collar Mode, e.g. Fence, Teach... */
	uint8_t eep_collar_mode;

	/** Current fence status, NotStarted, MaybeOutOfFence ... */
	uint8_t eep_fence_status;

	/** Current collar status, Stuck, OffAnimal, PowerOff .. */
	uint8_t eep_collar_status;

	/** If true, the collar has passed the "TeachMode" 
	  * test within the current fence definition.
	  */
	uint8_t eep_teach_mode_finished;

	/** Store the ble security key */
	uint8_t ble_sec_key[EEP_BLE_SEC_KEY_LEN];
	
	/** EMS Provider. */
	uint8_t eep_ems_provider;

	/* Generation of product */
	uint8_t eep_product_record_rev;
	
	/* Model of product */
	uint8_t eep_bom_mec_rev;

	/** Product Record version. */
	uint8_t eep_bom_pcb_rev;

	/** Hardware version. */
	uint8_t eep_hw_version;

	/** Product type; 1=Sheep/goat, 2=Cattle. */
	uint16_t eep_product_type;

	/** If we should keep teach mode or not on pasture update. */
	uint8_t eep_keep_mode;
};

#endif /* X3_FW_NF_SETTINGS_PRIVATE_H */
