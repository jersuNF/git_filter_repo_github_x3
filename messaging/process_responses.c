#include "process_responses.h"
#include "messaging_module_events.h"
#include "common.h"
#include "collar_protocol.h"
#include "messaging.h"

void process_poll_response(NofenceMessage *proto){
	PollMessageResponse *pResp = proto-> m.poll_message_resp;
	if (pResp->has_xServerIp && strlen(pResp->xServerIp) > 0) {
		messaging_host_address_event host_add_event =
			new_messaging_host_address_event();
		host_add_event->address = pResp->xServerIp;
		EVENT_SUBMIT(host_add_event); /*cellular controller writes it
 * to eeprom if it is different from the previously stored address.*/
		}
	if (pResp->has_bEraseFlash && pResp->bEraseFlash) {
		/* TODO: publish erase flash event to storage module */
	}
	// If we are asked to, reboot
	if (pResp->has_bReboot && pResp->bReboot) {
		/* TODO: publish reboot event to power manager */
	}
	/* TODO: set activation mode to (pResp->eActivationMode); */

	if (pResp->has_bUseUbloxAno) {
		/* TODO: publish enable ANO event to GPS controller */
	}
	if (pResp->has_bUseServerTime && pResp->bUseServerTime) {
		time_from_server = proto->header.ulUnixTimestamp;
	}
	if (pResp->has_usPollConnectIntervalSec) {
		poll_period_minutes = pResp->usPollConnectIntervalSec;
	}
	if (pResp->has_usAccSigmaSleepLimit) {
		/* TODO: submit pResp->usAccSigmaSleepLimit to AMC module.
		 * AMC will store it in eeprom or external flash. */
	}
	if (pResp->has_usAccSigmaNoActivityLimit) {
		/* TODO: submit pResp->usAccSigmaNoActivityLimit to AMC
		 * module. */
	}
	if (pResp->has_usOffAnimalTimeLimitSec) {
		/* TODO: submit pResp->usOffAnimalTimeLimitSec to AMC. */
	}
	if (pResp->has_rgubcBleKey) {
		/* TODO: submit pResp->rgubcBleKey.bytes,pResp->rgubcBleKey
		 * .size to BLE controller. */
	}
	if (pResp->has_versionInfo) {
		process_upgrade_request(pResp->versionInfo);
	}
	if (pResp->ulFenceDefVersion) {
		/* TODO: Submit pResp->ulFenceDefVersion to AMC */
		}
	}
	return;
}

/* @brief: starts a firmware download if a new version exists on the server. */
void process_upgrade_request(VersionInfoFW *fw_ver_from_server){
	// compare versions and start update when needed.//
	return;
}

void process_fence_msg(void){
	return;
}

void process_ano_msg(void){
	return;
}