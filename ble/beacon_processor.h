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
	uint8_t num_conn;
	uint8_t conn_history_peeker;
	bt_addr_le_t mac_address;
	uint8_t calculated_dist;
	struct beacon_connection_info history[CONFIG_BEACON_MAX_MEASUREMENTS];
};
struct beacon_list {
	uint8_t beacon_peeker;
	uint8_t arr_size;
	struct beacon_info beacon_array[CONFIG_BEACON_MAX_BROADCASTERS];
};

void init_beacon_list(void);

void beacon_process_event(uint32_t now_ms, const bt_addr_le_t *addr,
			  int8_t scanner_rssi_measured, adv_data_t *p_adv_data);

#endif /* BEACON_PROCESSOR_H */
