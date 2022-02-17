#include "build_proto_messages.h"
#include "messaging_module_events.h"
#include "common.h"

void build_poll_request(NofenceMessage *poll_req)
{
	proto_InitHeader(poll_req); /* fill up message header. */
	const gps_last_fix_struct_t *gpsLastFix = GPS_last_fix();
	poll_req.which_m = NofenceMessage_poll_message_req_tag;
	poll_req.m.poll_message_req.datePos = proto_getLastKnownDatePos(gpsLastFix);
	poll_req.m.poll_message_req.has_datePos =
		proto_hasLastKnownDatePos(gpsLastFix);
	poll_req.m.poll_message_req.eMode = EEPROM_GetMode();
	poll_req.m.poll_message_req.usZapCount = EEPROM_GetZapCnt();
	poll_req.m.poll_message_req.eCollarStatus = EEPROM_GetCollarStatus();
	poll_req.m.poll_message_req.eFenceStatus = EEPROM_GetFenceStatus();
	poll_req.m.poll_message_req.ulFenceDefVersion = EEPROM_GetFenceConfigCrc();
	poll_req.m.poll_message_req.usBatteryVoltage = readVbatt();
	poll_req.m.poll_message_req.has_ucMCUSR = (g_u8_mcusr_mirror != 0);
	poll_req.m.poll_message_req.ucMCUSR = g_u8_mcusr_mirror;
	const _GSM_INFO *p_gsm_info = bgs_get_gsm_info();
	poll_req.m.poll_message_req.xGsmInfo = *p_gsm_info;
	poll_req.m.poll_message_req.has_xGsmInfo = p_gsm_info->xMMC_MNC[0] != '\0';

	if (m_flash_erase_count != EEPROM_GetFlashEraseCount()) {
		// m_flash_erase_count is reset when we receive a poll reply
		poll_req.m.poll_message_req.has_usFlashEraseCount = true;
		poll_req.m.poll_message_req.usFlashEraseCount =
			EEPROM_GetFlashEraseCount();
		m_flash_erase_count =
			poll_req.m.poll_message_req.usFlashEraseCount - 1;
	}
	if (m_confirm_acc_limits) {
		poll_req.m.poll_message_req.has_usAccSigmaSleepLimit = true;
		poll_req.m.poll_message_req.usAccSigmaSleepLimit =
			EEPROM_GetAccSigmaSleepLimit();
		poll_req.m.poll_message_req.has_usAccSigmaNoActivityLimit = true;
		poll_req.m.poll_message_req.usAccSigmaNoActivityLimit =
			EEPROM_GetAccSigmaNoActivityLimit();
		poll_req.m.poll_message_req.has_usOffAnimalTimeLimitSec = true;
		poll_req.m.poll_message_req.usOffAnimalTimeLimitSec =
			EEPROM_GetOffAnimalTimeLimitSec();
	}
	if (m_confirm_ble_key) {
		poll_req.m.poll_message_req.has_rgubcBleKey = true;
		poll_req.m.poll_message_req.rgubcBleKey.size = EEP_BLE_SEC_KEY_LEN;
		EEPROM_ReadBleSecKey(poll_req.m.poll_message_req.rgubcBleKey.bytes,
				     EEP_BLE_SEC_KEY_LEN);
	}
	poll_req.m.poll_message_req.usGnssOnFixAgeSec = GPS_get_gnss_on_fix_age_sec(
		&poll_req.m.poll_message_req.has_usGnssOnFixAgeSec);
	poll_req.m.poll_message_req.usGnssTTFFSec = GPS_get_gnss_ttff_sec(
		&poll_req.m.poll_message_req.has_usGnssTTFFSec);

	if (m_transfer_boot_params ||
	    m_nrf_version_transfer_info == NRF_VERSION_TRANSFER_NEEDED) {
		poll_req.m.poll_message_req.has_versionInfo = true;
		if (m_transfer_boot_params) {
			uint16_t xbootVersion;
			if (xboot_get_version(&xbootVersion) == XB_SUCCESS) {
				poll_req.m.poll_message_req.versionInfo
					.usATmegaBootloaderVersion =
					xbootVersion;
				poll_req.m.poll_message_req.versionInfo
					.has_usATmegaBootloaderVersion = true;
			}
			poll_req.m.poll_message_req.has_versionInfoHW = true;
			poll_req.m.poll_message_req.versionInfoHW.ucPCB_RF_Version =
				EEPROM_GetHwVersion();
			poll_req.m.poll_message_req.versionInfoHW.usPCB_Product_Type =
				(uint8_t)EEPROM_GetProductType();
			poll_req.m.poll_message_req.has_xSimCardId = true;
			memcpy(poll_req.m.poll_message_req.xSimCardId, BGS_SCID(),
			       sizeof(poll_req.m.poll_message_req.xSimCardId));
			poll_req.m.poll_message_req.versionInfo.has_ulATmegaVersion =
				true;
			poll_req.m.poll_message_req.versionInfo.ulATmegaVersion =
				ATMEGA_1284P_FIRMWARE_VERSION;
		}
	}
}

void proto_InitHeader(NofenceMessage * msg) {
	msg->header.ulId = EEPROM_GetID();
	msg->header.ulVersion = ATMEGA_1284P_FIRMWARE_VERSION;
	msg->header.has_ulVersion = true; // TODO we actually only need to send the ulVersion after a reboot, in this case, the server can look up our version
	msg->header.ulUnixTimestamp = nf_get_unix_time();
}