#include "cmd_settings.h"

#include "nf_eeprom.h"

#include <string.h>

static int commander_settings_get_id(uint8_t* id, uint8_t* data, uint32_t size);
static int commander_settings_read(enum diagnostics_interface interface, settings_id_t id);
static int commander_settings_write(enum diagnostics_interface interface, settings_id_t id, uint8_t* data, uint32_t size);

int commander_settings_handler(enum diagnostics_interface interface, 
			       uint8_t cmd, uint8_t* data, uint32_t size)
{
	int err = 0;

	switch (cmd) {
		case READ:
		{
			uint8_t id = 0;
			if (commander_settings_get_id(&id, data, size) != 0) {
				commander_send_resp(interface, SETTINGS, cmd, NOT_ENOUGH, NULL, 0);
				break;
			}

			err = commander_settings_read(interface, id);
			
			break;
		}
		case WRITE:
		{
			uint8_t id = 0;
			if (commander_settings_get_id(&id, data, size) != 0) {
				commander_send_resp(interface, SETTINGS, cmd, NOT_ENOUGH, NULL, 0);
				break;
			}

			err = commander_settings_write(interface, id, &data[1], size-1);

			break;
		}

		case ERASE_ALL:
		default:
		{
			commander_send_resp(interface, SETTINGS, cmd, UNKNOWN_CMD, NULL, 0);
			err = -EINVAL;
		}
	}

	return err;
}

static int commander_settings_get_id(uint8_t* id, uint8_t* data, uint32_t size)
{
	if (size < 1) {
		return -EINVAL;
	}

	*id = data[0];

	return 0;
}

static int commander_settings_read(enum diagnostics_interface interface, settings_id_t id)
{
	int err = 0;

	uint8_t buf[50];
	buf[0] = id;

	switch (id) {
		case SERIAL:
		{
			uint32_t serial = 0;
			err = eep_read_serial(&serial);

			if (err == 0) {
				memcpy(&buf[1], &serial, sizeof(uint32_t));
				commander_send_resp(interface, SETTINGS, READ, DATA, buf, 1+sizeof(uint32_t));
			} else {
				commander_send_resp(interface, SETTINGS, READ, ERROR, NULL, 0);
			}
			break;
		}
		case HOST_PORT:
		default:
		{
			commander_send_resp(interface, SETTINGS, READ, UNKNOWN, NULL, 0);

			err = -EINVAL;
			break;
		}
	}

	return err;
}

static int commander_settings_write(enum diagnostics_interface interface, settings_id_t id, uint8_t* data, uint32_t size) 
{
	int err = 0;

	switch(id) {
		case SERIAL:
		{
			/* Must be exactly 4 bytes for uint32_t */
			if (size == 4) {
				uint32_t serial = (data[0]<<0) + 
						  (data[1]<<8) +
						  (data[2]<<16) +
						  (data[3]<<24);
				err = eep_write_serial(serial);
			}
			break;
		}
		case HOST_PORT:
		default:
		{
			err = -EINVAL;
			break;
		}
	}

	if (err == 0) {
		commander_send_resp(interface, SETTINGS, READ, ACK, NULL, 0);
	} else if (err == -EINVAL) {
		commander_send_resp(interface, SETTINGS, READ, UNKNOWN, NULL, 0);
	} else {
		commander_send_resp(interface, SETTINGS, READ, ERROR, NULL, 0);
	}

	return err;
}
