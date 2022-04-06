/*
 * Copyright (c) 2022 Nofence AS
 */

#include "commander.h"
#include "cobs.h"

#include "buzzer.h"
#include "nf_eeprom.h"

#include "sound_event.h"
#include "ep_event.h"
#include "event_manager.h"

#include "commander_def.h"

#include <stdbool.h>

static struct commander_action commander_actions;

uint8_t cobs_buffer[CONFIG_DIAGNOSTICS_RECEIVE_BUFFER_LENGTH+2];

static int commander_system_handler(enum diagnostics_interface interface, 
				      uint8_t cmd, uint8_t* data, uint32_t size);

static int commander_settings_handler(enum diagnostics_interface interface, 
				      uint8_t cmd, uint8_t* data, uint32_t size);

static int commander_stimulator_handler(enum diagnostics_interface interface, 
					uint8_t cmd, uint8_t* data, uint32_t size);

static int commander_storage_handler(enum diagnostics_interface interface, 
					uint8_t cmd, uint8_t* data, uint32_t size);

typedef struct group_handler_t {
	uint8_t group;
	int (*handler)(enum diagnostics_interface, uint8_t, uint8_t*, uint32_t);
}

const group_handler_t handlers[] = {
	{ .group = SYSTEM,
	  .handler = commander_system_handler},
	{ .group = SETTINGS,
	  .handler = commander_settings_handler},
	{ .group = STIMULATOR,
	  .handler = commander_stimulator_handler},
	{ .group = STORAGE,
	  .handler = commander_storage_handler},
}

int commander_init(struct commander_action* actions)
{
	if (actions != NULL) {
		commander_actions = *actions;
	}

	return 0;
}

static int commander_send_resp(enum diagnostics_interface interface, 
			       uint8_t group, uint8_t cmd, uint8_t resp,
			       uint8_t* data, uint8_t data_size)
{
	int ret = 0;

	commander_resp_t resp_ack = {
		.group = group,
		.command = cmd,
		.response = resp,
		.checksum = 0
	};

	uint8_t* buffer = NULL;
	uint32_t size = 0;

	bool need_freeing = false;

	if ((data != NULL) && (data_size != 0)) {
		size = sizeof(commander_resp_t) + data_size;
		buffer = k_malloc(size);
		need_freeing = true;
		
		memcpy(buffer, &resp_ack, sizeof(commander_resp_t));
		memcpy(&buffer[sizeof(commander_resp_t)], data, data_size);
	} else {
		size = sizeof(commander_resp_t);
		buffer = &resp_ack;
	}

	resp_ack.checksum = crc16_ccitt(0x0000, buffer, size);

	cobs_encode_result cobs_res;
	cobs_res = cobs_encode(cobs_buffer, sizeof(cobs_buffer)-1,
			       buffer, size);
	if (cobs_res.status == COBS_ENCODE_OK) {
		uint32_t packet_size = cobs_res.out_len;
		cobs_buffer[packet_size++] = '\x00';
		commander_actions.send_resp(interface, cobs_buffer, packet_size);
	} else {
		ret = -ECOMM;
	}

	if (need_freeing) {
		k_free(buffer);
	}

	return ret;
}

uint32_t commander_handle(enum diagnostics_interface interface, 
			  uint8_t* data, uint32_t size)
{
	/* Look for 0 that marks end of COBS packet */
	uint32_t packet_end = 0;
	while (packet_end < size) {
		if (data[packet_end] == 0) {
			break;
		}
		packet_end++;
	}

	if (packet_end >= size) {
		/* No completed COBS packet found */
		return 0;
	}

	/* Use COBS to decode packet data */
	cobs_decode_result cobs_res;
	cobs_res = cobs_decode(cobs_buffer, sizeof(cobs_buffer),
			       data, packet_end);

	/* Take action according to data */
	if ((cobs_res.status == COBS_DECODE_OK) && (cobs_res.out_len >= sizeof(commander_header_t))) {
		
		commander_header_t* header = cobs_buffer;
		uint32_t packet_size = cobs_res.out_len;

		uint8_t* data_buffer = &cobs_buffer[sizeof(commander_header_t)]
		uint32_t data_size = cobs_res.out_len-sizeof(commander_header_t);

		/* Verify checksum */
		uint16_t checksum = header->checksum;
		header_checksum = 0x0000;
		
		if (checksum == crc16_ccitt(0x0000, (uint8_t*)header, packet_size)) {
			/** Call group handler */
			group_handler_t *hndl = NULL;
			for (int i = 0; i < sizeof(handlers)/sizeof(group_handler_t); i++) {
				if (header->group == handlers[i].group) {
					hndl = &handlers[i];
					break;
				}
			}
			if (hndl != NULL) {
				hndl->handler(interface, 
					header->command, 
					data_buffer, 
					data_size);
			} else {
				/* Unknown group, send error */
				commander_send_resp(interface, header->group, header->command, UNKNOWN_GRP, NULL, 0);
			}
		} else {
			/* Checksum failed, send error */
			commander_send_resp(interface, header->group, header->command, CHK_FAILED, NULL, 0);
		}
	}

	/* We have consumed data up to and including the 0 delimiter */
	return packet_end+1;
}

static int commander_settings_get_id(uint8_t* data, uint32_t size)
{
	if (size < 1) {
		return -EINVAL;
	}

	return data[0];
}

static int commander_settings_read(uint8_t id, uint8_t* data, uint32_t size)
{
	int ret = 0;

	return ret;
}

static int commander_settings_handler(enum diagnostics_interface interface, 
				      uint8_t cmd, uint8_t* data, uint32_t size)
{
	int err = 0;

	switch (cmd) {
		case READ:
			uint8_t id = 0;
			if (commander_settings_get_id(data, size) != 0) {
				commander_send_resp(interface, SETTINGS, cmd, NOT_ENOUGH, NULL, 0);
				break;
			}

			
			uint32_t serial = 0;
			err = eep_read_serial(&serial);
			if (err == 0) {
				commander_send_data(interface, 
							COMMANDER_CMD_SETTINGS,
							(uint8_t*)&serial, 
							sizeof(serial));
			}
			break;

			/* Must be exactly 4 bytes for uint32_t */
				if ((size-2) == 4) {
					uint32_t serial = (data[2+0]<<0) + 
							  (data[2+1]<<8) +
							  (data[2+2]<<16) +
							  (data[2+3]<<24);
					err = eep_write_serial(serial);
				}
		
		case WRITE:

		case ERASE_ALL:
			


		default:
		{
			err = -EINVAL;
		}
	}

cleanup:
	return err;
}

static int commander_stimulator_handler(enum diagnostics_interface interface, 
				      uint8_t cmd, uint8_t* data, uint32_t size)
{
	int err = 0;
	
	uint8_t resp = ACK;

	switch (cmd) {
		case GNSS_DATA:
			/** @todo Process and send GNSS data event */
			break;
		case BUZZER_WARN:
			/** @todo Warning frequency as argument? */

			/* Simulate the highest tone event */
			struct sound_event *sound_event_warn = new_sound_event();
			sound_event_warn->type = SND_WARN;
			EVENT_SUBMIT(sound_event_warn);
			
			struct sound_set_warn_freq_event *sound_warn_freq = new_sound_set_warn_freq_event();
			sound_warn_freq->freq = WARN_FREQ_MS_PERIOD_MAX;
			EVENT_SUBMIT(sound_warn_freq);

			break;
		case EP_RELEASE:
			/* Send electric pulse */
			struct ep_status_event *ready_ep_event = new_ep_status_event();
			ready_ep_event->ep_status = EP_RELEASE;
			EVENT_SUBMIT(ready_ep_event);

			break;
		default:
			resp = UNKNOWN_CMD;
			break;
	}

	commander_send_resp(interface, STIMULATOR, cmd, resp, NULL, 0);

cleanup:
	return err;
}