#include "cmd_modem.h"
#include "modem_nf.h"
#include <stddef.h>
#include <string.h>

int commander_modem_handler(enum diagnostics_interface interface, uint8_t cmd, uint8_t *data,
			    uint32_t size)
{
	int err = 0;

	uint8_t resp = ACK;

	switch (cmd) {
	case GET_CCID: {
		char *ccid = NULL;

		err = get_ccid(&ccid);
		if (err == 0) {
			commander_send_resp(interface, MODEM, cmd, DATA, (uint8_t *)ccid,
					    strlen(ccid));
		} else {
			commander_send_resp(interface, MODEM, cmd, ERROR, NULL, 0);
		}
		break;
	}
	default:
		resp = UNKNOWN_CMD;
		commander_send_resp(interface, MODEM, cmd, resp, NULL, 0);
		break;
	}

	return err;
}
