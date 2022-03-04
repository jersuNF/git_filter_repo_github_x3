/*
 * Copyright (c) 2022 Nofence AS
 */

#include "commander.h"
#include "cobs.h"

#include "sound_event.h"
#include "ep_event.h"
#include "event_manager.h"

#include <stdbool.h>

static struct commander_action commander_actions;

uint8_t cobs_buffer[CONFIG_DIAGNOSTICS_RECEIVE_BUFFER_LENGTH+2];

int commander_init(struct commander_action* actions)
{
	if (actions != NULL) {
		commander_actions = *actions;
	}

	return 0;
}

static int commander_ack(enum diagnostics_interface interface, bool ack, uint8_t cmd)
{
	int ret = 0;

	uint8_t resp_ack[] = {'N', cmd, ack ? 0x01 : 0x00};

	cobs_encode_result cobs_res;
	cobs_res = cobs_encode(cobs_buffer, sizeof(cobs_buffer)-1,
			       resp_ack, sizeof(resp_ack));
	if (cobs_res.status == COBS_ENCODE_OK) {
		uint32_t size = cobs_res.out_len;
		cobs_buffer[size++] = '\x00';
		commander_actions.send_resp(interface, cobs_buffer, size);
	} else {
		ret = -ECOMM;
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
	if ((cobs_res.status == COBS_DECODE_OK) && (cobs_res.out_len > 0)) {
		/* TODO - Specify and implement proper commander format */
		bool ack = false;
		uint8_t cmd = 0;
		if (cobs_buffer[0] == 'N') {
			if (cobs_buffer[1] == 0x20) {
				/* Simulate the highest tone event */
				struct sound_event *sound_event_high = new_sound_event();
				sound_event_high->type = SND_MAX;
				EVENT_SUBMIT(sound_event_high);

				cmd = 0x20;
				ack = true;
			} else if (cobs_buffer[1] == 0x50) {
				/* Send electric pulse */
				struct ep_status_event *ready_ep_event = new_ep_status_event();
				ready_ep_event->ep_status = EP_RELEASE;
				EVENT_SUBMIT(ready_ep_event);
				
				cmd = 0x50;
				ack = true;
			}
		}
		commander_ack(interface, ack, cmd);
	}

	/* We have consumed data up to and including the 0 delimiter */
	return packet_end+1;
}
