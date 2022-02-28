#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
#include <math.h>
#include <sys/ring_buffer.h>
#include <logging/log.h>

#include "beacon_processor.h"

LOG_MODULE_REGISTER(beacon_processor);

/** @brief defines the minimum number of RSSI measurements to define a RSSI based distance */
#define MIN_VALID_BEACON_MEASUREMENTS 5

/** @brief : Contains the whole structure for the tracked beacons **/
static struct beacon_list beacons;

/**
 * @brief Function to converts an unsigned 16 bit RSSI value to a signed int8 
 * @param[in] x uint16_t RSSI value
 * @return int8 converted RSSI value
 */
static inline int8_t signed2(uint16_t x)
{
	uint8_t x1 = (uint8_t)x;
	uint8_t x2 = ~x1;
	uint16_t x3 = x2 + (uint16_t)1;
	return (int8_t)-x3;
}

/**
 * @brief Function check if two ble mac address are equal
 * @param[in] p1 Pointer to bt_addr_le_t address to compare
 * @param[in] p2 Pointer to bt_addr_le_t address to compare
 * @return True if addresses are equal, otherwise false
 */
static inline bool is_equal_mac_addr(const bt_addr_le_t *p1,
				     const bt_addr_le_t *p2)
{
	return (bt_addr_le_cmp(p1, p2) == 0);
}

/**
 * @brief Function convert the beacon mac address to a string array.
 * @param[out] s Pointer to char array to store the string
 * @param[in] len Length of the char array
 * @param[in] addr bt_addr_le_t beacon address
 * @return Pointer to char array holding the beacon MAC ADDR.
 */
static char *mac2string(char *s, size_t len, const bt_addr_le_t *addr)
{
	static const char *fmt_upper = "%02x:%02x:%02x:%02x:%02x:%02x";
	snprintf(s, len, fmt_upper, addr->a.val[5], addr->a.val[4],
		 addr->a.val[3], addr->a.val[2], addr->a.val[1],
		 addr->a.val[0]);

	return s;
}

/**
 * @brief Function to add a beacon to the beacon list
 * @param[out] list Pointer to beacon_list
 * @param[in] src Pointer to beacon_info struct
 * @return index to beacon in beacon list, -1 if Beacon is not found
 */
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

/**
 * @brief Function to add a beacon to the beacon list
 * @param[out] list Pointer to beacon_list
 * @param[in] src Pointer to beacon_info struct
 */
static inline void add_to_beacon_list(struct beacon_list *list,
				      struct beacon_info *src)
{
	int ret = get_beacon_index_by_mac(list, src);
	if (ret < 0) {
		struct beacon_info *dst =
			&list->beacon_array[list->beacon_peeker];
		memcpy(dst, src, sizeof(struct beacon_info));

		list->beacon_peeker++;
		if (list->beacon_peeker >= CONFIG_BEACON_MAX_BROADCASTERS) {
			list->beacon_peeker = 0;
		}

		if (list->arr_size < CONFIG_BEACON_MAX_BROADCASTERS) {
			list->arr_size++;
		}
	} else {
		struct beacon_info *existing_beacon = &list->beacon_array[ret];
		memcpy(existing_beacon, src, sizeof(struct beacon_info));
	}
}
/**
 * @brief Function to add beacon connection info to a beacon_info struct.
 * @param[in] info Pointer to beacon_connection_info struct
 * @param[out] beacon Pointer to beacon_info struct
 */
static inline void add_to_beacon_history(struct beacon_connection_info *info,
					 struct beacon_info *beacon)
{
	struct beacon_connection_info *dst =
		&beacon->history[beacon->conn_history_peeker];
	memcpy(dst, info, sizeof(struct beacon_connection_info));
	beacon->conn_history_peeker++;
	if (beacon->conn_history_peeker >= CONFIG_BEACON_MAX_MEASUREMENTS) {
		beacon->conn_history_peeker = 0;
	}
	if (beacon->num_conn < CONFIG_BEACON_MAX_MEASUREMENTS) {
		beacon->num_conn++;
	}
}

#ifdef CONFIG_BEACON_SHORTEST_DISTANCE
/**
 * @brief Function to set the shortest distance, from measurement array.
 * @param[in] beacon Pointer to beacon info struct
 * @return 0 on sucess. Otherwise -ENODATA
 */
static int set_new_shortest_dist(struct beacon_info *beacon)
{
	/* Initialize to largest possible distance */
	uint32_t shortest_dist = 255;
	uint8_t entries = 0;

	for (uint8_t i = 0; i < beacon->num_conn; i++) {
		struct beacon_connection_info *info = &beacon->history[i];
		if (k_uptime_get_32() - info->time_diff <
		    CONFIG_BEACON_MAX_MEASUREMENT_AGE * MSEC_PER_SEC) {
			if (info->beacon_dist < 1) {
				continue;
			}
			if (info->beacon_dist < shortest_dist) {
				shortest_dist = info->beacon_dist;
			}
			entries++;
		}
	}

	if (entries == 0) {
		return -ENODATA;
	}

	beacon->avg_dist = shortest_dist;

	return 0;
}
#else
/**
 * @brief Function to set the average distance
 * @param[in] beacon Pointer to beacon info struct
 * @return 0 on sucess. Otherwise -ENODATA
 */
static int set_new_avg_dist(struct beacon_info *beacon)
{
	uint32_t avg = 0;
	uint8_t entries = 0;

	for (uint8_t i = 0; i < beacon->num_conn; i++) {
		struct beacon_connection_info *info = &beacon->history[i];
		if (k_uptime_get_32() - info->time_diff <
		    CONFIG_BEACON_MAX_MEASUREMENT_AGE * MSEC_PER_SEC) {
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
#endif

/**
 * @brief Function to get the shortest average distance.
 * 
 * @param[in] list struct of beacon_list
 * @param[out] dist Distance pointer
 * @param[out] beacon_index Pointer to index in beacon array
 * @return 0 on sucess. Else a negative error code.
 */
static inline int get_shortest_avg_distance(struct beacon_list *list,
					    uint8_t *dist,
					    uint8_t *beacon_index)
{
	int err;
	*dist = UINT8_MAX;

	for (uint8_t i = 0; i < list->arr_size; i++) {
		/* After new connection entry has been added, 
		 * we have a new average. 
		 */
		struct beacon_info *c_info = &list->beacon_array[i];

#ifdef CONFIG_BEACON_SHORTEST_DISTANCE
		/* Update the shortest distance across last 10 second
		 * readings 
		 */
		err = set_new_shortest_dist(c_info);
		if (err) {
			LOG_ERR("Error calculating shortest distance");
		}
#else
		/* Update the average on the beacons, based on 
		 * time since recorded distance. 
		 */
		err = set_new_avg_dist(c_info);
		if (err) {
			LOG_ERR("Error calculating average distance");
		}
#endif
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
 * @brief Function to calculate the distance to a beacon based 
 * 	  on tx power and received rssi strength.
 * 
 * @param[in] tx_power Broadcasted tx power, from advertised payload
 * @param[in] rssi Measured RSSI value. 
 * @return Distance in meter
 */
static double calculate_accuracy(int8_t tx_power, int8_t rssi)
{
	if (rssi == 0) {
		LOG_ERR("Cannot detirmine rssi value");
		return DBL_MAX;
	}
	double ratio = rssi / (double)tx_power;
	if (ratio < 1.0) {
		return pow(ratio, 10.0);
	} else {
		double accuracy = (0.89976) * pow(ratio, 7.7095) + 0.111;
		return accuracy;
	}
}

/**
 * @brief Function called in scan callback to process a nearby beacon
 * 
 * @param[in] now_ms Uptime in milliseconds
 * @param[in] addr Pointer to beacon bt_addr_le_t address
 * @param[in] scanner_rssi_measured RSSI value measured
 * @param[in] p_adv_data Pointer to beacon advertise data
 * @return True if beacon is successfully processed. 
 */
bool beacon_process_event(uint32_t now_ms, const bt_addr_le_t *addr,
			  int8_t scanner_rssi_measured, adv_data_t *p_adv_data)
{
	int8_t beacon_adv_rssi = signed2(p_adv_data->rssi);
	double m = calculate_accuracy(beacon_adv_rssi, scanner_rssi_measured);

	if (m > CONFIG_BEACON_DISTANCE_MAX || m > BEACON_DISTANCE_INFINITY) {
		return false;
	}

	if (m < 1.0) {
		m = 1.0;
	}

	char mac_current[MAC_CHARBUF_SIZE];
	LOG_INF("Device=%s  RX_RSSI=%d TX_RSSI=%d  m=%d",
		log_strdup(mac2string(mac_current, sizeof(mac_current), addr)),
		(int)scanner_rssi_measured, (int)beacon_adv_rssi, (int)m);

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
		beacon.avg_dist = UINT8_MAX;
		beacon.num_conn = 0;
		beacon.conn_history_peeker = 0;
		add_to_beacon_history(&info, &beacon);
		add_to_beacon_list(&beacons, &beacon);
	} else {
		add_to_beacon_history(&info,
				      &beacons.beacon_array[target_beacon]);
	}

	uint8_t shortest_dist;
	uint8_t beacon_index = 0;
	int err = get_shortest_avg_distance(&beacons, &shortest_dist,
					    &beacon_index);

	if (err) {
		LOG_ERR("No data in array, err: %d", err);
		return false;
	}

	char mac_best[MAC_CHARBUF_SIZE];
	LOG_INF("Calculated new avg shortest distance %u from Beacon_%u: %s",
		shortest_dist, beacon_index,
		log_strdup(mac2string(
			mac_best, sizeof(mac_best),
			&beacons.beacon_array[beacon_index].mac_address)));

	return true;
}

/**
 * @brief Function to initialize the beacon list 
 */
void init_beacon_list(void)
{
	memset(&beacons, 0, sizeof(beacons));
	for (uint8_t i = 0; i < CONFIG_BEACON_MAX_BROADCASTERS; i++) {
		memset(&beacons.beacon_array[i], 0, sizeof(struct beacon_info));
	}
}
