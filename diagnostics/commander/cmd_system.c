#include "cmd_system.h"
#include <stddef.h>

int commander_system_handler(enum diagnostics_interface interface, 
			     uint8_t cmd, uint8_t* data, uint32_t size)
{
	int err = 0;
	
	uint8_t resp = ACK;

	switch (cmd) {
		case PING:
		{
			commander_send_resp(interface, SYSTEM, cmd, DATA, data, size);
			break;
		}
		case REBOOT:
		{
			resp = ACK;
			/** @todo Schedule reboot after 1s */
			commander_send_resp(interface, SYSTEM, cmd, resp, NULL, 0);
			break;
		}
		default:
			resp = UNKNOWN_CMD;
			commander_send_resp(interface, SYSTEM, cmd, resp, NULL, 0);
			break;
	}

	return err;
}
