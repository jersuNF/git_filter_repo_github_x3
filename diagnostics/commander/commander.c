/*
 * Copyright (c) 2022 Nofence AS
 */

#include "commander.h"
#include "cobs.h"

#include <sys/crc.h>

#include "commander_def.h"

#include "cmd_system.h"
#include "cmd_settings.h"
#include "cmd_stimulator.h"
#include "cmd_storage.h"

#include <stdbool.h>
#include <string.h>

static struct commander_action commander_actions;

uint8_t cobs_buffer[CONFIG_DIAGNOSTICS_RECEIVE_BUFFER_LENGTH+2];

int commander_system_handler(enum diagnostics_interface interface, 
				      uint8_t cmd, uint8_t* data, uint32_t size);


int commander_stimulator_handler(enum diagnostics_interface interface, 
					uint8_t cmd, uint8_t* data, uint32_t size);

int commander_storage_handler(enum diagnostics_interface interface, 
					uint8_t cmd, uint8_t* data, uint32_t size);

typedef struct {
	uint8_t group;
	int (*handler)(enum diagnostics_interface, uint8_t, uint8_t*, uint32_t);
} group_handler_t;

const group_handler_t handlers[] = {
	{ .group = SYSTEM,
	  .handler = commander_system_handler},
	{ .group = SETTINGS,
	  .handler = commander_settings_handler},
	{ .group = STIMULATOR,
	  .handler = commander_stimulator_handler},
	{ .group = STORAGE,
	  .handler = commander_storage_handler},
};

int commander_init(struct commander_action* actions)
{
	if (actions != NULL) {
		commander_actions = *actions;
	}

	return 0;
}

int commander_send_resp(enum diagnostics_interface interface, 
			       commander_group_t group, uint8_t cmd, 
			       commander_resp_t resp,
			       uint8_t* data, uint8_t data_size)
{
	int err = 0;

	commander_resp_header_t resp_ack = {
		.group = group,
		.command = cmd,
		.response = resp,
		.checksum = 0
	};

	uint8_t* buffer = NULL;
	uint32_t size = 0;

	bool need_freeing = false;

	if ((data != NULL) && (data_size != 0)) {
		size = sizeof(commander_resp_header_t) + data_size;
		buffer = k_malloc(size);
		need_freeing = true;
		
		memcpy(buffer, &resp_ack, sizeof(commander_resp_header_t));
		memcpy(&buffer[sizeof(commander_resp_header_t)], data, data_size);
	} else {
		size = sizeof(commander_resp_header_t);
		buffer = (uint8_t*)&resp_ack;
	}

	commander_resp_header_t* header = (commander_resp_header_t*)buffer;
	header->checksum = crc16_ccitt(0x0000, buffer, size);

	cobs_encode_result cobs_res;
	cobs_res = cobs_encode(cobs_buffer, sizeof(cobs_buffer)-1,
			       buffer, size);
	if (cobs_res.status == COBS_ENCODE_OK) {
		uint32_t packet_size = cobs_res.out_len;
		cobs_buffer[packet_size++] = '\x00';
		commander_actions.send_resp(interface, cobs_buffer, packet_size);
	} else {
		err = -ECOMM;
	}

	if (need_freeing) {
		k_free(buffer);
	}

	return err;
}

uint32_t commander_handle(enum diagnostics_interface interface, 
			  uint8_t* data, uint32_t size)
{
	uint32_t parsed_bytes = 0;

	while (parsed_bytes < size) {
		/* Look for 0 that marks end of COBS packet */
		uint32_t packet_end = parsed_bytes;
		while (packet_end < size) {
			if (data[packet_end] == 0) {
				break;
			}
			packet_end++;
		}
		uint32_t bytes_in_packet = packet_end - parsed_bytes;

		if (packet_end >= size) {
			/* No completed COBS packet found */
			break;
		}

		/* Use COBS to decode packet data */
		cobs_decode_result cobs_res;
		cobs_res = cobs_decode(cobs_buffer, sizeof(cobs_buffer),
				&data[parsed_bytes], bytes_in_packet);

		/* Take action according to data */
		if ((cobs_res.status == COBS_DECODE_OK) && (cobs_res.out_len >= sizeof(commander_cmd_header_t))) {
			
			commander_cmd_header_t* header = (commander_cmd_header_t*)cobs_buffer;
			uint32_t packet_size = cobs_res.out_len;

			uint8_t* data_buffer = &cobs_buffer[sizeof(commander_cmd_header_t)];
			uint32_t data_size = cobs_res.out_len-sizeof(commander_cmd_header_t);

			/* Verify checksum */
			uint16_t checksum = header->checksum;
			header->checksum = 0x0000;
			
			if (checksum == crc16_ccitt(0x0000, (uint8_t*)header, packet_size)) {
				/** Call group handler */
				const group_handler_t *hndl = NULL;
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
		parsed_bytes += bytes_in_packet+1;
	}

	/* We have consumed data up to and including the 0 delimiter */
	return parsed_bytes;
}
