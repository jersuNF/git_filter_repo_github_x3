#include "cmd_stimulator.h"

#include "buzzer.h"
#include "sound_event.h"
#include "ep_event.h"
#include "event_manager.h"

#include "gnss_hub.h"

int commander_stimulator_handler(enum diagnostics_interface interface, 
				 uint8_t cmd, uint8_t* data, uint32_t size)
{
	int err = 0;
	
	uint8_t resp = ACK;

	switch (cmd) {
		case GNSS_HUB:
		{
			if (size != 1) {
				resp = ERROR;
				break;
			}

			uint8_t hub_mode = data[0];
			if (gnss_hub_configure(hub_mode) != 0) {
				resp = ERROR;
			}
			
			break;
		}
		case GNSS_SEND:
		{
			if (gnss_hub_send(GNSS_HUB_ID_DIAGNOSTICS, data, size) != 0) {
				resp = ERROR;
			}
			break;
		}
		case BUZZER_WARN:
		{
			/** @todo Warning frequency as argument? */

			/* Simulate the highest tone event */
			struct sound_event *sound_event_warn = new_sound_event();
			sound_event_warn->type = SND_WARN;
			EVENT_SUBMIT(sound_event_warn);
			
			struct sound_set_warn_freq_event *sound_warn_freq = new_sound_set_warn_freq_event();
			sound_warn_freq->freq = WARN_FREQ_MS_PERIOD_MAX;
			EVENT_SUBMIT(sound_warn_freq);

			break;
		}
		case ELECTRICAL_PULSE:
		{
			/* Send electric pulse */
			struct ep_status_event *ready_ep_event = new_ep_status_event();
			ready_ep_event->ep_status = EP_RELEASE;
			EVENT_SUBMIT(ready_ep_event);

			break;
		}
		default:
			resp = UNKNOWN_CMD;
			break;
	}

	commander_send_resp(interface, STIMULATOR, cmd, resp, NULL, 0);

	return err;
}