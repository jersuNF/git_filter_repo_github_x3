/*
 * Copyright (c) 2021 Nofence AS
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
#include <math.h>
#include <sys/ring_buffer.h>

#define MODULE beacon_processor
#include <logging/log.h>

#include "beacon_processor.h"
#include "ble_beacon_event.h"

LOG_MODULE_REGISTER(MODULE, CONFIG_BEACON_PROCESSOR_LOG_LEVEL);
/** @brief : Contains the whole structure for the tracked beacons **/
static struct beacon_list beacons;

/** @brief : Save the last calculated distance for hysteresis */
static uint8_t last_calculated_distance;
typedef enum {
	CROSS_UNDEFINED = 0,
	CROSS_LOW_FROM_BELOW,
	CROSS_HIGH_FROM_ABOVE
} cross_type_t;

/** @brief : Used for hysteresis calculation **/
static cross_type_t cross_type = CROSS_UNDEFINED;

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
	if (beacon->num_measurements < CONFIG_BEACON_MAX_MEASUREMENTS) {
		beacon->num_measurements++;
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
	uint8_t shortest_dist = UINT8_MAX;
	uint8_t entries = 0;
	for (uint8_t i = 0; i < beacon->num_measurements; i++) {
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

	beacon->calculated_dist = shortest_dist;

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

	for (uint8_t i = 0; i < beacon->num_measurements; i++) {
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

	beacon->calculated_dist = avg / entries;
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
static inline int get_shortest_distance(struct beacon_list *list, uint8_t *dist,
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
		if (err == -ENODATA) {
			LOG_WRN("Age of all measurements from beacon_%u are larger than 10 seconds",
				i);
			continue;
		}
#else
		/* Update the average on the beacons, based on 
		 * time since recorded distance. 
		 */
		err = set_new_avg_dist(c_info);
		if (err == -ENODATA) {
			LOG_WRN("Age of all measurements from beacon_%u are larger than 10 seconds",
				i);
			continue;
		}
#endif
		if (c_info->calculated_dist < *dist) {
			*dist = c_info->calculated_dist;
			*beacon_index = i;
		}
	}

	if (*dist == UINT8_MAX) {
		/* No elements in list, or age of all elements are larger than age limit */
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
		LOG_WRN("Cannot detirmine RSSI value");
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

int beacon_process_event(uint32_t now_ms, const bt_addr_le_t *addr,
			 int8_t scanner_rssi_measured, adv_data_t *p_adv_data)
{
	int8_t beacon_adv_rssi = signed2(p_adv_data->rssi);
	double m = calculate_accuracy(beacon_adv_rssi, scanner_rssi_measured);

	if (m > CONFIG_BEACON_DISTANCE_MAX || m > BEACON_DISTANCE_INFINITY) {
		last_calculated_distance = UINT8_MAX;
		/* Beacon is found but out of desired range */
		struct ble_beacon_event *event_err = new_ble_beacon_event();
		event_err->status = BEACON_STATUS_OUT_OF_RANGE;
		EVENT_SUBMIT(event_err);
		return -EIO;
	}

	if (m < 1.0) {
		m = 1.0;
	}

	char mac_current[MAC_CHARBUF_SIZE];
	LOG_DBG("Device=%s  RX_RSSI=%d TX_RSSI=%d  m=%d",
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
		beacon.calculated_dist = UINT8_MAX;
		beacon.num_measurements = 0;
		beacon.conn_history_peeker = 0;
		add_to_beacon_history(&info, &beacon);
		add_to_beacon_list(&beacons, &beacon);
	} else {
		add_to_beacon_history(&info,
				      &beacons.beacon_array[target_beacon]);
	}

	uint8_t shortest_dist;
	uint8_t beacon_index = 0;
	int err =
		get_shortest_distance(&beacons, &shortest_dist, &beacon_index);

	if (err < 0) {
		LOG_WRN("No data in array, err: %d", err);
		shortest_dist = UINT8_MAX;
	} else {
		char mac_best[MAC_CHARBUF_SIZE];
		LOG_DBG("Calculated new shortest distance %u m from Beacon_%u: %s",
			shortest_dist, beacon_index,
			log_strdup(
				mac2string(mac_best, sizeof(mac_best),
					   &beacons.beacon_array[beacon_index]
						    .mac_address)));
	}

	if (shortest_dist == UINT8_MAX) {
		struct ble_beacon_event *event = new_ble_beacon_event();
		cross_type = CROSS_UNDEFINED;
		event->status = BEACON_STATUS_NOT_FOUND;
		LOG_DBG("1: Status: BEACON_STATUS_NOT_FOUND, Type: CROSS_UNDEFINED");
		EVENT_SUBMIT(event);

	} else if (shortest_dist > CONFIG_BEACON_HIGH_LIMIT) {
		struct ble_beacon_event *event = new_ble_beacon_event();
		cross_type = CROSS_UNDEFINED;
		event->status = BEACON_STATUS_REGION_FAR;
		LOG_DBG("2: Status: BEACON_STATUS_REGION_FAR, Type: CROSS_UNDEFINED");
		EVENT_SUBMIT(event);

	} else if (shortest_dist <= CONFIG_BEACON_LOW_LIMIT) {
		struct ble_beacon_event *event = new_ble_beacon_event();
		cross_type = CROSS_UNDEFINED;
		event->status = BEACON_STATUS_REGION_NEAR;
		LOG_DBG("3: Status: BEACON_STATUS_REGION_NEAR, Type: CROSS_UNDEFINED");
		EVENT_SUBMIT(event);

	} else if (last_calculated_distance <= CONFIG_BEACON_LOW_LIMIT &&
		   shortest_dist > CONFIG_BEACON_LOW_LIMIT) {
		struct ble_beacon_event *event = new_ble_beacon_event();
		cross_type = CROSS_LOW_FROM_BELOW;
		event->status = BEACON_STATUS_REGION_NEAR;
		LOG_DBG("4: Status: BEACON_STATUS_REGION_NEAR, Type: CROSS_LOW_FROM_BELOW");
		EVENT_SUBMIT(event);

	} else if (last_calculated_distance > CONFIG_BEACON_HIGH_LIMIT &&
		   shortest_dist <= CONFIG_BEACON_HIGH_LIMIT) {
		struct ble_beacon_event *event = new_ble_beacon_event();
		cross_type = CROSS_HIGH_FROM_ABOVE;
		event->status = BEACON_STATUS_REGION_FAR;
		LOG_DBG("5: Status: BEACON_STATUS_REGION_FAR, Type: CROSS_HIGH_FROM_ABOVE");
		EVENT_SUBMIT(event);

	} else {
		if (cross_type == CROSS_LOW_FROM_BELOW) {
			struct ble_beacon_event *event = new_ble_beacon_event();
			event->status = BEACON_STATUS_REGION_NEAR;
			LOG_DBG("6: Status: BEACON_STATUS_REGION_NEAR, Type: CROSS_LOW_FROM_BELOW");
			EVENT_SUBMIT(event);

		} else if (cross_type == CROSS_HIGH_FROM_ABOVE) {
			struct ble_beacon_event *event = new_ble_beacon_event();
			event->status = BEACON_STATUS_REGION_FAR;
			LOG_DBG("7: Status: BEACON_STATUS_REGION_FAR, Type: CROSS_HIGH_FROM_ABOVE");
			EVENT_SUBMIT(event);

		} else {
			// Cross type undefined
			LOG_ERR("Unecspected state, last calculated: %u, shortest: %d",
				last_calculated_distance, shortest_dist);
			last_calculated_distance = shortest_dist;
			return -EIO;
		}
	}
	last_calculated_distance = shortest_dist;
	return shortest_dist;
}

void init_beacon_list(void)
{
	memset(&beacons, 0, sizeof(beacons));
	for (uint8_t i = 0; i < CONFIG_BEACON_MAX_BROADCASTERS; i++) {
		memset(&beacons.beacon_array[i], 0, sizeof(struct beacon_info));
	}
}
