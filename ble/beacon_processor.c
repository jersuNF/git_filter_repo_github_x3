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
				      struct beacon_info *src, uint8_t m)
{
	int index = get_beacon_index_by_mac(list, src);

	/* Beacon not found. Check first if list is full */
	if (list->num_beacons >= CONFIG_BEACON_MAX_BROADCASTERS) {
		/* Beacon list is full. Remove beacon with worst measurement */
		uint8_t worst_distance = 0;
		for (int i = 0; i < CONFIG_BEACON_MAX_BROADCASTERS; i++) {
			struct beacon_info *tmp = &list->beacon_array[i];
			if (tmp->calculated_dist > worst_distance) {
				worst_distance = tmp->calculated_dist;
				index = i;
			}
		}
		if (m > worst_distance || m == UINT8_MAX) {
			/* Check if we try to add something worse than already added */
			return;
		}

		LOG_DBG("Replace the worst beacon in list with incomming beacon");
		struct beacon_info *dst = &list->beacon_array[index];
		memset(dst, 0, sizeof(struct beacon_info));
		memcpy(dst, src, sizeof(struct beacon_info));
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
	// printk("[ "); /* Uncomment for debug */
	for (uint8_t i = 0; i < beacon->num_measurements; i++) {
		struct beacon_connection_info *info = &beacon->history[i];
		// printk("%d ", info->beacon_dist); /* Uncomment for debug */
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
	// printk("]\n"); /* Uncomment for debug */

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

	for (uint8_t i = 0; i < list->num_beacons; i++) {
		/* After new connection entry has been added, 
		 * we have a new average. 
		 */
		struct beacon_info *c_info = &list->beacon_array[i];

		/* Create string of beacon address */
		char beacon_str[BT_ADDR_STR_LEN];
		bt_addr_le_to_str(&c_info->mac_address, beacon_str,
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
	double m = calculate_accuracy(beacon_adv_rssi, scanner_rssi_measured);

	if (m > CONFIG_BEACON_DISTANCE_MAX || m > BEACON_DISTANCE_INFINITY) {
		/* A beacon is seen, but out of desired range. Do not add to list */
		last_calculated_distance = UINT8_MAX;
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
	/* If it doesn't exit, add it to list. */
	if (target_beacon == -1) {
		LOG_DBG("New beacon detected %s", log_strdup(beacon_str));
		/* Populate 1 new history entry. */
		beacon.calculated_dist = UINT8_MAX;
		beacon.num_measurements = 0;
		beacon.conn_history_peeker = 0;
		add_to_beacon_history(&info, &beacon);
		add_to_beacon_list(&beacons, &beacon, (uint8_t)m);
		return -EIO;
	} else {
		LOG_DBG("Add measurement from beacon %s to list",
			log_strdup(beacon_str));
		add_to_beacon_history(&info,
				      &beacons.beacon_array[target_beacon]);
	}
	if (beacons.beacon_array[target_beacon].num_measurements < 4) {
		/* Wait for at least four measurements to evaluate a beacon */
		return -EIO;
	}
	uint8_t shortest_dist;
	uint8_t beacon_index = 0;
	int err =
		get_shortest_distance(&beacons, &shortest_dist, &beacon_index);

	if (err < 0) {
		LOG_WRN("No data in array, err: %d", err);
		shortest_dist = UINT8_MAX;
	} else {
		/* Create string of beacon address */
		char beacon_str[BT_ADDR_STR_LEN];
		bt_addr_le_to_str(
			&beacons.beacon_array[beacon_index].mac_address,
			beacon_str, sizeof(beacon_str));
		LOG_DBG("Use shortest distance %u m from Beacon: %s",
			shortest_dist, log_strdup(beacon_str));
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
			return -EPERM;
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
