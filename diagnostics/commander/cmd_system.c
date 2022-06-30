#include "cmd_system.h"
#include "selftest.h"
#include "log_backend_diag.h"
#include <stddef.h>
#include <string.h>

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
		case LOG:
		{
			if (size < 1) {
				resp = NOT_ENOUGH;
				commander_send_resp(interface, SYSTEM, cmd, resp, NULL, 0);
				err = -EINVAL;
			} else {
				if (data[0]) {
					log_backend_diag_enable(interface);
				} else {
					log_backend_diag_disable();
				}
				resp = ACK;
				commander_send_resp(interface, SYSTEM, cmd, resp, NULL, 0);
			}
			break;
		}
		case TEST:
		{
			uint32_t test_buf[2];
			selftest_get_result(&test_buf[0], &test_buf[1]);

			commander_send_resp(interface, SYSTEM, cmd, DATA, (uint8_t*)test_buf, sizeof(test_buf));
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
