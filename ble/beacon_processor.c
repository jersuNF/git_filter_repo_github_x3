#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
#include <math.h>
#include <sys/ring_buffer.h>
#include <logging/log.h>

#include "beacon_processor.h"

LOG_MODULE_REGISTER(beacon_processor);

RING_BUF_DECLARE(beacon_ring_buf, BEACON_RING_BUF_SIZE);

/** @brief defines the minimum number of RSSI measurements to define a RSSI based distance */
#define MIN_VALID_BEACON_MEASUREMENTS 5

/** @brief : Contains the whole structure for the tracked beacons **/
static struct beacon_list beacons;

/** @brief converts an unsigned 16 bit RSSI value to a signed int8 */
static inline int8_t signed2(uint16_t x)
{
	uint8_t x1 = (uint8_t)x;
	uint8_t x2 = ~x1;
	uint16_t x3 = x2 + (uint16_t)1;
	return (int8_t)-x3;
}

static inline bool is_equal_mac_addr(const bt_addr_le_t *p1,
				     const bt_addr_le_t *p2)
{
	return (bt_addr_le_cmp(p1, p2) == 0);
}

static char *mac2string(char *s, size_t sSize, const bt_addr_le_t *pMac)
{
	static const char *fmt_upper = "%02x:%02x:%02x:%02x:%02x:%02x";
	snprintf(s, sSize, fmt_upper, pMac->a.val[5], pMac->a.val[4],
		 pMac->a.val[3], pMac->a.val[2], pMac->a.val[1],
		 pMac->a.val[0]);

	return s;
}
static inline int get_beacon_index_by_mac(struct beacon_list *list,
					  struct beacon_info *info)
{
	for (uint8_t i = 0; i < list->arr_size; i++) {
		if (is_equal_mac_addr(&list->beacon_array[i].mac_address,
				      &info->mac_address)) {
			return i;
		}
	}
	return -1;
}

static inline int add_to_beacon_list(struct beacon_list *list,
				     struct beacon_info *src)
{
	int ret = get_beacon_index_by_mac(list, src);
	if (ret < 0) {
		struct beacon_info *dst =
			&list->beacon_array[list->beacon_peeker];
		memcpy(dst, src, sizeof(struct beacon_info));

		list->beacon_peeker++;
		if (list->beacon_peeker >= MAX_BEACONS) {
			list->beacon_peeker = 0;
		}

		if (list->arr_size < MAX_BEACONS) {
			list->arr_size++;
		}
	} else {
		struct beacon_info *existing_beacon = &list->beacon_array[ret];
	}
}

static inline void add_to_beacon_history(struct beacon_connection_info *info,
					 struct beacon_info *beacon)
{
	struct beacon_connection_info *dst =
		&beacon->history[beacon->conn_history_peeker];
	memcpy(dst, info, sizeof(struct beacon_connection_info));
	beacon->conn_history_peeker++;
	if (beacon->conn_history_peeker >= MAX_BEACON_MEASUREMENTS) {
		beacon->conn_history_peeker = 0;
	}
	if (beacon->num_conn < MAX_BEACON_MEASUREMENTS) {
		beacon->num_conn++;
	}
}

/**
 * @brief Parser handle function will parse provided data and call requested actions. 
 * 
 * @param[in] list Interface from where the data was received. 
 * @param[out] dist Buffer of data. 
 * @param[out] beacon_index Size of data. 
 * 
 * @return Number of bytes parsed in data buffer. 
 */
static inline int get_shortest_avg_distance(struct beacon_list *list,
					    uint8_t *dist,
					    uint8_t *beacon_index)
{
	*dist = UINT8_MAX;

	for (uint8_t i = 0; i < list->arr_size; i++) {
		/* After new connection entry has been added, 
		 * we have a new average. 
		 */
		struct beacon_info *c_info = &list->beacon_array[i];

		/* Update the average on the beacons, based on 
		 * time since recorded distance.
		 */
		int err = set_new_avg_dist(c_info);
		if (err) {
			LOG_ERR("Error calculating average distance");
		}

		if (c_info->avg_dist < dist) {
			*dist = c_info->avg_dist;
			*beacon_index = i;
		}
	}

	if (*dist == UINT8_MAX) {
		return -ENODATA;
	}
	return 0;
}
/**
https://github.com/RadiusNetworks/android-ibeacon-service/blob/4185a5bd0c657acaf145098a09466bb34a144557/src/main/java/com/radiusnetworks/ibeacon/IBeacon.java
 TODO, pow is too large, create a lookup table from ratio instead
*/

static double calculateAccuracy(int8_t txPower, int8_t rssi)
{
	if (rssi == 0) {
		LOG_ERR("Cannot detirmine rssi");
		return DBL_MAX; // if we cannot determine accuracy, return
	}
	double ratio = rssi / (double)txPower;
	//LOG_INF("Ratio: %f", ratio);
	if (ratio < 1.0) {
		return pow(ratio, 10.0);
	} else {
		double accuracy = (0.89976) * pow(ratio, 7.7095) + 0.111;
		//LOG_INF("Accuracy: %f", accuracy);
		return accuracy;
	}
}

static int set_new_avg_dist(struct beacon_info *beacon)
{
	uint32_t avg = 0;
	uint8_t entries = 0;

	for (uint8_t i = 0; i < beacon->num_conn; i++) {
		struct beacon_connection_info *info = &beacon->history[i];
		if (k_uptime_get_32() - info->time_diff <
		    MAX_MEASUREMENT_AGE_SEC * MSEC_PER_SEC) {
			if (info->beacon_dist < 1) {
				continue;
			}

			avg += info->beacon_dist;
			entries++;
		}
	}

	if (entries == 0) {
		return -ENODATA;
	}

	beacon->avg_dist = avg / entries;

	return 0;
}

bool beacon_process_event(uint32_t now_ms, const bt_addr_le_t *addr,
			  int8_t rssi_sample_value, adv_data_t *p_adv_data)
{
	int8_t beacon_adv_rssi = signed2(p_adv_data->rssi);
	double m = calculateAccuracy(beacon_adv_rssi, rssi_sample_value);

	if (m > BEACON_DIST_MAX_M || m > BEACON_DIST_INFINITY) {
		return false;
	}

	if (m < 1.0) {
		m = 1.0;
	}

	struct beacon_connection_info info;
	info.beacon_dist = (uint8_t)m;
	info.time_diff = now_ms;

	struct beacon_info beacon;

	/* Copy address. */
	memcpy(&beacon.mac_address, addr, sizeof(bt_addr_le_t));

	int target_beacon = get_beacon_index_by_mac(&beacons, &beacon);

	/* If it doesn't exit, add it to list. */
	if (target_beacon == -1) {
		/* Populate 1 new history entry. */
		memset(&beacon.history, 0, sizeof(beacon.history));
		add_to_beacon_history(&info, &beacon);
		add_to_beacon_list(&beacons, &beacon);
	} else {
		add_to_beacon_history(&info,
				      &beacons.beacon_array[target_beacon]);
	}

	uint8_t shortest_dist;
	uint8_t beacon_index;
	int err = get_shortest_avg_distance(&beacons, &shortest_dist,
					    &beacon_index);

	if (err) {
		return false;
	}

	char mac_best[MAC_CHARBUF_SIZE];
	LOG_INF("Calculated new avg shortest distance %d from beacon %s",
		(int)m,
		log_strdup(mac2string(
			mac_best, sizeof(mac_best),
			beacons.beacon_array[beacon_index].mac_address)));

	char mac_current[MAC_CHARBUF_SIZE];
	LOG_INF("Device=%s  RX_RSSI=%d TX_RSSI=%d  m=%d",
		log_strdup(mac2string(mac_current, sizeof(mac_current), addr)),
		(int)rssi_sample_value, (int)beacon_adv_rssi, (int)m);

	return true;
}

void init_beacon_list(void)
{
	memset(&beacons, 0, sizeof(beacons));
	for (uint8_t i = 0; i < MAX_BEACONS; i++) {
		memset(&beacons.beacon_array[i], 0, sizeof(struct beacon_info));
	}
}
