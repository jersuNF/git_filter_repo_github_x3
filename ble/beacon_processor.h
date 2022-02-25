#ifndef BEACON_PROCESSOR_H
#define BEACON_PROCESSOR_H

#include <bluetooth/addr.h>
#include "beacon.h"

#define BEACON_RING_BUF_SIZE (2048 * 2)

#define MAC_CHARBUF_SIZE (37 + 8)

/**@brief Max age of measurements in the RSSI fifo buffer. If a measurement is older than
 * this, it will be ignored.
 * */
#define MAX_MEASUREMENT_AGE_SEC 10

/**@brief Max Beacons which can be maintained at the same time, must be a multiple of 2 */
#define MAX_BEACONS 4

/**@brief Ring buffer size of beacon measurements, must be a multiple of 2 */
#define MAX_BEACON_MEASUREMENTS 8

#define RING_BUFFER_MASK (MAX_BEACON_MEASUREMENTS - 1)

#define BEACON_DIST_NOT_SET (UINT8_MAX)
#define TIME_DIFF_NOT_SET (UINT8_MAX)

#define BEACON_DIST_INFINITY (UINT8_MAX - 1)
#define BEACON_DIST_MAX_M (50)

struct beacon_connection_info {
	uint8_t beacon_dist;
	uint8_t time_diff;
};

struct beacon_info {
	size_t num_conn;
	uint8_t conn_history_peeker;
	bt_addr_le_t mac_address;
	uint8_t calculated_dist;
	uint32_t prevMeasureTime;
	struct beacon_connection_info history[MAX_BEACON_MEASUREMENTS];
};
struct beacon_list {
	uint8_t beacon_peeker;
	size_t arr_size;
	struct beacon_info beacon_array[MAX_BEACONS];
};

void beac_init(void);

void beac_reset(void);

bool beac_process_event(uint32_t now_ms, const bt_addr_le_t *addr,
			int8_t rssi_sample_value, adv_data_t *p_adv_data);

SingleBeaconInfo_t *beac_get_nearest_beacon(uint32_t minTimestamp);

#endif
