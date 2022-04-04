/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include <stdlib.h>
#include <string.h>
#include "messaging_module_events.h"
#include "messaging.h"
#include <logging/log.h>
#include "ble_ctrl_event.h"
#include "ble_data_event.h"
#include "ble_cmd_event.h"
#include "nofence_service.h"
#include "lte_proto_event.h"

#include "cellular_controller_events.h"
#include "gnss_controller_events.h"
#include "request_events.h"
#include "nf_version.h"
#include "collar_protocol.h"

#include "error_event.h"
#include "helpers.h"
#include <power/reboot.h>

#include "nf_crc16.h"

#include "storage_event.h"

#include "storage.h"

#include "pasture_structure.h"
#include "fw_upgrade_events.h"
#include "sound_event.h"

K_KERNEL_STACK_DEFINE(update_durations_stack,
		      TIME_USE_THREAD_STACK_SIZE);
struct k_thread update_durations;

struct histograms
{
	static _HISTOGRAM_ANIMAL_BEHAVE animal_behave;
	static _HISTOGRAM_ZONE in_zone;
	static _POSITION_QC_MAX_MIN_MEAN qc_baro_gps_max_mean_min;
	static _HISTOGRAM_CURRENT_PROFILE current_profile;
	static _BATTERY_QC qc_battery;
};

int time_use_module_init(void)
{
	LOG_INF("Initializing time_use module.");

	k_thread_create(&update_durations, update_durations_stack,
			K_KERNEL_STACK_SIZEOF(keep_alive_stack),
			(k_thread_entry_t) update_elapsed_time,
			NULL, NULL, NULL,
			K_PRIO_COOP(TIME_USE_THREAD_PRIORITY),
			0, K_NO_WAIT);

	return 0;
}

void update_elapsed_time(void){
	while (true){
		;
	}
}