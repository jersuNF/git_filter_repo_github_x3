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

typedef struct _BeaconHistory {
	uint8_t beacon_dist;
	uint8_t time_diff;
} BeaconHistory;

/** @brief : Very simple ring buffer to store measurements for one Beacon **/
struct ring_buffer_t {
	/** Buffer memory. */
	BeaconHistory buffer[MAX_BEACON_MEASUREMENTS];
	uint8_t tail_index;
	uint8_t head_index;
};

typedef struct _SingleBeaconInfo {
	bt_addr_le_t mac_address;
	uint8_t calculated_dist;
	uint32_t prevMeasureTime;
	struct ring_buffer_t beacon_hist_fifo;

} SingleBeaconInfo_t;

typedef struct _BeaconsInfo {
	SingleBeaconInfo_t single_beacon_infos[MAX_BEACONS];
} BeaconsInfo_t;

void beac_init(void);

void beac_reset(void);

bool beac_process_event(uint32_t now_ms, const bt_addr_le_t *addr,
			int8_t rssi_sample_value, adv_data_t *p_adv_data);

SingleBeaconInfo_t *beac_get_nearest_beacon(uint32_t minTimestamp);

#endif
