/*
 * Copyright (c) 2021 Nofence AS
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
#include <math.h>
#include <sys/ring_buffer.h>
#include <logging/log.h>

#include "beacon_processor.h"

#define MODULE beacon_processor
LOG_MODULE_REGISTER(MODULE, CONFIG_BEACON_PROCESSOR_LOG_LEVEL);

/** @brief : Contains the whole structure for all the tracked beacons **/
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
 * @brief Function to add a beacon to the beacon list
 * @param[out] list Pointer to beacon_list
 * @param[in] src Pointer to beacon_info struct
 * @return index to beacon in beacon list, -1 if Beacon is not found
 */
static inline int get_beacon_index_by_mac(struct beacon_list *list,
					  struct beacon_info *info)
{
	for (uint8_t i = 0; i < list->num_beacons; i++) {
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
 * @param[in] m Measured distance to the beacon we want to add
 */
static inline void add_to_beacon_list(struct beacon_list *list,
				      struct beacon_info *src)
{
	int index = -1;

	/* Beacon not found. Check first if list is full */
	if (list->num_beacons >= CONFIG_BEACON_MAX_BROADCASTERS) {
		/* Beacon list is full. Remove beacon with worst measurement */
		uint8_t worst_distance = 0;
		for (int i = 0; i < CONFIG_BEACON_MAX_BROADCASTERS; i++) {
			struct beacon_info *tmp = &list->beacon_array[i];
			if (tmp->min_dist > worst_distance) {
				worst_distance = tmp->min_dist;
				index = i;
			}
		}
		if (src->min_dist > worst_distance || src->min_dist == UINT8_MAX) {
			/* Check if we try to add something worse than already added */
			return;
		}
		if (index > -1) {
			LOG_DBG("Replace the worst beacon in list with incomming beacon");
			struct beacon_info *dst = &list->beacon_array[index];
			memset(dst, 0, sizeof(struct beacon_info));
			memcpy(dst, src, sizeof(struct beacon_info));
		}

	} else {
		/* Space available. Add beacon to list */
		struct beacon_info *dst =
			&list->beacon_array[list->num_beacons];
		memset(dst, 0, sizeof(struct beacon_info));
		memcpy(dst, src, sizeof(struct beacon_info));
		/* Increment to next available slot */
		list->num_beacons++;
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
	beacon->history[beacon->conn_history_peeker].beacon_dist =
		info->beacon_dist;
	beacon->history[beacon->conn_history_peeker].time_diff =
		info->time_diff;
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
	uint8_t shortest_dist = UINT8_MAX;
	uint8_t entries = 0;

	for (uint8_t i = 0; i < beacon->num_measurements; i++) {
		struct beacon_connection_info *info = &beacon->history[i];
		int32_t delta_t = k_uptime_get_32() - info->time_diff;
//		LOG_WRN("delta_t = %d", delta_t);
		if (((delta_t >= 0) && (delta_t <
		    CONFIG_BEACON_MAX_MEASUREMENT_AGE * MSEC_PER_SEC)) ||
		    ((delta_t < 0) && ((uint32_t)(delta_t + UINT32_MAX) <
				      (CONFIG_BEACON_MAX_MEASUREMENT_AGE *
					       MSEC_PER_SEC)))) {

			if ((info->beacon_dist >= 1) && 
			    (info->beacon_dist < shortest_dist)) {
				shortest_dist = info->beacon_dist;
				entries++;
			}
		}
	}

	beacon->min_dist = shortest_dist;
	if (beacon->min_dist == UINT8_MAX) {
		return -ENODATA;
	}
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

	beacon->min_dist = avg / entries;
	return 0;
}
#endif

int beacon_shortest_distance(uint8_t *dist)
{
	int err;
	*dist = UINT8_MAX;

	for (uint8_t i = 0; i < beacons.num_beacons; i++) {
		struct beacon_info *c_info = &beacons.beacon_array[i];

		/* Create string of beacon address */
		char beacon_str[BT_ADDR_STR_LEN];
		bt_addr_le_to_str(&(c_info->mac_address), beacon_str,
				  sizeof(beacon_str));

		/* Uncomment below for debug */
		// printk("Beacon %s: ", log_strdup(beacon_str));
#ifdef CONFIG_BEACON_SHORTEST_DISTANCE
		/* Update the shortest distance across last 10 second
		 * readings 
		 */
		err = set_new_shortest_dist(c_info);
		if (err == -ENODATA) {
			LOG_WRN("Age of all measurements from beacon %s are larger than 10 seconds",
				log_strdup(beacon_str));
			continue;
		}
#else
		/* Update the average on the beacons, based on 
		 * time since recorded distance. 
		 */
		err = set_new_avg_dist(c_info);
		if (err == -ENODATA) {
			LOG_WRN("Age of all measurements from beacon %s are larger than 10 seconds",
				log_strdup(beacon_str));
			continue;
		}
#endif
		if (c_info->min_dist < *dist) {
			*dist = c_info->min_dist;
		}
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
	if (rssi == 0 || tx_power == 0) {
		LOG_WRN("Cannot detirmine RSSI value");
		return DBL_MAX;
	}
	double ratio = rssi / (double)tx_power;
	// https://www.researchgate.net/publication/278021046_Measuring_a_Distance_between_Things_with_Improved_Accuracy
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
	int8_t rssi_sample_value = -1 * scanner_rssi_measured;
	double m = calculate_accuracy(beacon_adv_rssi, rssi_sample_value);

	if (m > CONFIG_BEACON_DISTANCE_MAX) {
		/* A beacon is seen, but out of desired range. Do not add to list */
		return -EIO;
	}

	if (m < 1.0) {
		m = 1.0;
	}

	struct beacon_connection_info info;
	info.beacon_dist = (uint8_t)m;
	info.time_diff = now_ms;

	struct beacon_info beacon;

	/* Copy address into struct */
	memcpy(&beacon.mac_address, addr, sizeof(bt_addr_le_t));

	/* Create string of beacon address */
	char beacon_str[BT_ADDR_STR_LEN];
	bt_addr_le_to_str(&beacon.mac_address, beacon_str, sizeof(beacon_str));

	int target_beacon = get_beacon_index_by_mac(&beacons, &beacon);
	/* If beacon doesn't exist (-1), add it to list. */
	if (target_beacon == -1) {
		LOG_DBG("New beacon detected %s, %d", 
			log_strdup(beacon_str), (uint8_t)m);

		/* Populate 1 new entry in beacon history. */
		beacon.min_dist = (uint8_t)m;
		beacon.num_measurements = 0;
		beacon.conn_history_peeker = 0;
		add_to_beacon_history(&info, &beacon);

		/* Add beacon to beacons list */
		add_to_beacon_list(&beacons, &beacon);
	} else {
		LOG_DBG("Add measurement (%d) from beacon %s to list",
			(uint8_t)m, log_strdup(beacon_str));
		add_to_beacon_history(&info,
				&beacons.beacon_array[target_beacon]);
	}
	return 0;
}

void init_beacon_list(void)
{
	/* Reset the beacons list at initialization */
	memset(&beacons, 0, sizeof(beacons));
}
