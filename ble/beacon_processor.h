/*
 * Copyright (c) 2021 Nofence AS
 */

#ifndef BEACON_PROCESSOR_H
#define BEACON_PROCESSOR_H

#include <bluetooth/addr.h>
#include "beacon.h"

#define MAC_CHARBUF_SIZE (37 + 8)
#define BEACON_DISTANCE_INFINITY (UINT8_MAX - 1)

struct beacon_connection_info {
	uint8_t beacon_dist;
	uint32_t time_diff;
};

struct beacon_info {
	uint8_t num_measurements;
	uint8_t conn_history_peeker;
	bt_addr_le_t mac_address;
	uint8_t calculated_dist;
	struct beacon_connection_info history[CONFIG_BEACON_MAX_MEASUREMENTS];
};
struct beacon_list {
	uint8_t num_beacons;
	struct beacon_info beacon_array[CONFIG_BEACON_MAX_BROADCASTERS];
};

/**
 * @brief Function to initialize the beacon list 
 */
void init_beacon_list(void);

/**
 * @brief Function called in scan callback to process a nearby beacon
 * 
 * @param[in] now_ms Uptime in milliseconds
 * @param[in] addr Pointer to beacon bt_addr_le_t address
 * @param[in] scanner_rssi_measured RSSI value measured
 * @param[in] p_adv_data Pointer to beacon advertise data
 * @return shortest distance, -EIO if measurement is out of range or -EPERM if process failed.
 */
int beacon_process_event(uint32_t now_ms, const bt_addr_le_t *addr,
			 int8_t scanner_rssi_measured, adv_data_t *p_adv_data);

#endif /* BEACON_PROCESSOR_H */
