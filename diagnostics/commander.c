/*
 * Copyright (c) 2022 Nofence AS
 */

#include "commander.h"
#include "cobs.h"

#include "sound_event.h"
#include "nf_settings.h"

#include "sound_event.h"
#include "ep_event.h"
#include "event_manager.h"

#include "log_backend_diag.h"

#include <stdbool.h>

static struct commander_action commander_actions;

uint8_t cobs_buffer[CONFIG_DIAGNOSTICS_RECEIVE_BUFFER_LENGTH + 2];

static int commander_settings_handler(enum diagnostics_interface interface,
				      uint8_t *data, uint32_t size);

int commander_init(struct commander_action *actions)
{
	if (actions != NULL) {
		commander_actions = *actions;
	}

	return 0;
}

static int commander_send_ack(enum diagnostics_interface interface, bool ack,
			      uint8_t cmd)
{
	int ret = 0;

	uint8_t resp_ack[] = { 'N', 'A', cmd, ack ? 0x01 : 0x00 };

	cobs_encode_result cobs_res;
	cobs_res = cobs_encode(cobs_buffer, sizeof(cobs_buffer) - 1, resp_ack,
			       sizeof(resp_ack));
	if (cobs_res.status == COBS_ENCODE_OK) {
		uint32_t size = cobs_res.out_len;
		cobs_buffer[size++] = '\x00';
		commander_actions.send_resp(interface, cobs_buffer, size);
	} else {
		ret = -ECOMM;
	}

	return ret;
}

static int commander_send_data(enum diagnostics_interface interface,
			       uint8_t cmd, uint8_t *data, uint8_t data_size)
{
	int ret = 0;

	uint8_t resp_data[256 + 4];
	resp_data[0] = 'N';
	resp_data[1] = 'D';
	resp_data[2] = cmd;
	resp_data[3] = data_size;
	memcpy(&resp_data[4], data, data_size);

	cobs_encode_result cobs_res;
	cobs_res = cobs_encode(cobs_buffer, sizeof(cobs_buffer) - 1, resp_data,
			       data_size + 4);
	if (cobs_res.status == COBS_ENCODE_OK) {
		uint32_t size = cobs_res.out_len;
		cobs_buffer[size++] = '\x00';
		commander_actions.send_resp(interface, cobs_buffer, size);
	} else {
		ret = -ECOMM;
	}

	return ret;
}

#define COMMANDER_CMD_WARN_MAX		0x20
#define COMMANDER_CMD_RELEASE_EP	0x50
#define COMMANDER_CMD_SETTINGS		0x70
#define COMMANDER_CMD_LOGGING		0x99

uint32_t commander_handle(enum diagnostics_interface interface, uint8_t *data,
			  uint32_t size)
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
	cobs_res =
		cobs_decode(cobs_buffer, sizeof(cobs_buffer), data, packet_end);

	/* Take action according to data */
	if ((cobs_res.status == COBS_DECODE_OK) && (cobs_res.out_len >= 2)) {
		/* TODO - Specify and implement proper commander format */
		bool ack = false;
		uint8_t cmd = 0;
		if (cobs_buffer[0] == 'N') {
			if (cobs_buffer[1] == COMMANDER_CMD_LOGGING) {
				if (cobs_buffer[2] == 1) {
					log_backend_diag_enable(interface);
				} else {
					log_backend_diag_disable();
				}

				cmd = COMMANDER_CMD_LOGGING;
				ack = true;
			} else if (cobs_buffer[1] == COMMANDER_CMD_WARN_MAX) {
				/* Simulate the highest tone event */
				struct sound_event *sound_event_warn =
					new_sound_event();
				sound_event_warn->type = SND_WARN;
				EVENT_SUBMIT(sound_event_warn);

				struct sound_set_warn_freq_event
					*sound_warn_freq =
						new_sound_set_warn_freq_event();
				sound_warn_freq->freq = WARN_FREQ_MAX;
				EVENT_SUBMIT(sound_warn_freq);

				cmd = COMMANDER_CMD_WARN_MAX;
				ack = true;
			} else if (cobs_buffer[1] == COMMANDER_CMD_RELEASE_EP) {
				/* Send electric pulse */
				struct ep_status_event *ready_ep_event =
					new_ep_status_event();
				ready_ep_event->ep_status = EP_RELEASE;
				EVENT_SUBMIT(ready_ep_event);

				cmd = COMMANDER_CMD_RELEASE_EP;
				ack = true;
			} else if (cobs_buffer[1] == COMMANDER_CMD_SETTINGS) {
				/* Settings API */
				if (cobs_res.out_len >= 3) {
					cmd = COMMANDER_CMD_SETTINGS;
					if (commander_settings_handler(
						    interface, &cobs_buffer[2],
						    cobs_res.out_len - 2) ==
					    0) {
						ack = true;
					}
				}
			}
		}
		commander_send_ack(interface, ack, cmd);
	}

	/* We have consumed data up to and including the 0 delimiter */
	return packet_end + 1;
}

#define COMMANDER_SETTINGS_SERIAL 0

static int commander_settings_handler(enum diagnostics_interface interface,
				      uint8_t *data, uint32_t size)
{
	int err = 0;

	uint8_t settings_id = data[0];
	bool write_request = size > 1;

	uint8_t write_length = 0;
	if (write_request) {
		write_length = data[1];
		if (size != (2 + write_length)) {
			err = -EINVAL;
			goto cleanup;
		}
	}

	switch (settings_id) {
	case COMMANDER_SETTINGS_SERIAL: {
		if (write_request) {
			/* Must be exactly 4 bytes for uint32_t */
			if ((size - 2) == 4) {
				uint32_t serial = (data[2 + 0] << 0) +
						  (data[2 + 1] << 8) +
						  (data[2 + 2] << 16) +
						  (data[2 + 3] << 24);
				err = eep_uint32_write(EEP_UID, serial);
			}
		} else {
			uint32_t serial = 0;
			err = eep_uint32_read(EEP_UID, &serial);
			if (err == 0) {
				commander_send_data(interface,
						    COMMANDER_CMD_SETTINGS,
						    (uint8_t *)&serial,
						    sizeof(serial));
			}
		}
	}
	default: {
		err = -EINVAL;
	}
	}

cleanup:
	return err;
}
