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
static BeaconsInfo_t m_beaconsInfo;

static inline uint8_t ring_buffer_num_items(struct ring_buffer_t *buffer)
{
	return ((buffer->head_index - buffer->tail_index) & RING_BUFFER_MASK);
}

static inline uint8_t ring_buffer_peek(struct ring_buffer_t *buffer,
				       BeaconHistory *data, uint8_t index)
{
	if (index >= ring_buffer_num_items(buffer)) {
		return 0;
	}
	/* Add index to pointer */
	uint8_t data_index = ((buffer->tail_index + index) & RING_BUFFER_MASK);
	*data = buffer->buffer[data_index];
	return 1;
}

static inline uint8_t ring_buffer_is_full(struct ring_buffer_t *buffer)
{
	return ((buffer->head_index - buffer->tail_index) & RING_BUFFER_MASK) ==
	       RING_BUFFER_MASK;
}

static inline uint8_t ring_buffer_is_empty(struct ring_buffer_t *buffer)
{
	return (buffer->head_index == buffer->tail_index);
}

static inline void ring_buffer_queue(struct ring_buffer_t *buffer,
				     BeaconHistory *data)
{
	/* Is buffer full? */
	if (ring_buffer_is_full(buffer)) {
		/* Is going to overwrite the oldest byte */
		/* Increase tail index */
		buffer->tail_index =
			((buffer->tail_index + 1) & RING_BUFFER_MASK);
	}

	/* Place data in buffer */
	buffer->buffer[buffer->head_index] = *data;
	buffer->head_index = ((buffer->head_index + 1) & RING_BUFFER_MASK);
}

static uint8_t ring_buffer_dequeue(struct ring_buffer_t *buffer,
				   BeaconHistory *data)
{
	if (ring_buffer_is_empty(buffer)) {
		/* No items */
		return 0;
	}

	*data = buffer->buffer[buffer->tail_index];
	buffer->tail_index = ((buffer->tail_index + 1) & RING_BUFFER_MASK);
	return 1;
}

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

/**
https://github.com/RadiusNetworks/android-ibeacon-service/blob/4185a5bd0c657acaf145098a09466bb34a144557/src/main/java/com/radiusnetworks/ibeacon/IBeacon.java
 TODO, pow is too large, create a lookup table from ratio instead
*/

static double calculateAccuracy(int8_t txPower, int8_t rssi)
{
	if (rssi == 0) {
		//LOG_ERR("Cannot detirmine rssi");
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

static uint8_t get_median_dist(SingleBeaconInfo_t *pB, uint32_t now)
{
	uint32_t delta = now - pB->prevMeasureTime;
	//LOG_INF("prev measure time: %u, delta: %u", pB->prevMeasureTime, delta);
	BeaconHistory hist;
	pB->calculated_dist = BEACON_DIST_INFINITY;

	uint16_t fifo_index = ring_buffer_num_items(&pB->beacon_hist_fifo) - 1;
	while (delta <= MAX_MEASUREMENT_AGE_SEC &&
	       ring_buffer_peek(&pB->beacon_hist_fifo, &hist, fifo_index)) {
		fifo_index--;
		delta += hist.time_diff;
		//LOG_INF("hist.beacon_dist: %u", hist.time_diff);

		if (hist.beacon_dist < pB->calculated_dist) {
			pB->calculated_dist = hist.beacon_dist;
		}
	}
	//LOG_INF("pb->calculated_dist %u", pB->calculated_dist);
	return pB->calculated_dist;
}

static void reset_beacon_info_values(SingleBeaconInfo_t *pBeaconInfo)
{
	memset(&pBeaconInfo->beacon_hist_fifo, 0,
	       sizeof(pBeaconInfo->beacon_hist_fifo));
	pBeaconInfo->calculated_dist = BEACON_DIST_INFINITY;
}

static void reset_beacon_info(SingleBeaconInfo_t *pBeaconInfo)
{
	memset(pBeaconInfo, 0, sizeof(*pBeaconInfo));
	reset_beacon_info_values(pBeaconInfo);
}

SingleBeaconInfo_t *beac_get_nearest_beacon(uint32_t now)
{
	uint8_t minDist = BEACON_DIST_INFINITY;
	uint8_t ind = UINT8_MAX;
	for (uint8_t i = 0; i < MAX_BEACONS; i++) {
		uint8_t dist = get_median_dist(
			&m_beaconsInfo.single_beacon_infos[i], now);
		//LOG_INF("Dist: %u", dist);
		if (dist < minDist) {
			minDist = dist;
			ind = i;
		}
	}
	return ind < MAX_BEACONS ? &m_beaconsInfo.single_beacon_infos[ind] :
				   NULL;
}

/** @brief : Searches the FIFO buffer of MAC addresses and finds or overwrites.
 * In all cases, this MAC will be marked as the most current */
static SingleBeaconInfo_t *
queue_beacon_info_head(const bt_addr_le_t *p_mac_addr, uint8_t m, uint32_t now)
{
	uint8_t index = UINT8_MAX;
	bool foundExisting = false;

	for (uint8_t i = 0; i < MAX_BEACONS; i++) {
		if (is_equal_mac_addr(
			    &m_beaconsInfo.single_beacon_infos[i].mac_address,
			    p_mac_addr)) {
			index = i;
			//LOG_INF("MAC address is equal, index is %u ", index);
			foundExisting = true;
			break;
		}
	}
	//LOG_INF("index is %u ", index);
	// New beacon, replace the beacon with worst measurements and worse than m. In this case
	// we must update the cached distances
	if (index == UINT8_MAX) {
		uint8_t worstDistance = 0;
		char mac_s[MAC_CHARBUF_SIZE];
		// Update distances
		(void)beac_get_nearest_beacon(now);
		for (uint8_t i = 0; i < MAX_BEACONS; i++) {
			//LOG_ERR("%d MAC=%s dist=%d\n", i,
			// log_strdup(mac2string(
			// 	mac_s, sizeof(mac_s),
			// 	&m_beaconsInfo.single_beacon_infos[i]
			// 		 .mac_address)),
			// m_beaconsInfo.single_beacon_infos[i]
			// 	.calculated_dist);

			if (m_beaconsInfo.single_beacon_infos[i]
				    .calculated_dist > worstDistance) {
				index = i;
				worstDistance =
					m_beaconsInfo.single_beacon_infos[i]
						.calculated_dist;
			}
		}
		if (m > worstDistance) {
			return NULL;
		}
	}

	if (!foundExisting) {
		char mac_s[MAC_CHARBUF_SIZE];
		char mac_s1[MAC_CHARBUF_SIZE];
		// LOG_INF("RESETS beacon info %s replaces %s (%d)\n",
		// 	log_strdup(
		// 		mac2string(mac_s, sizeof(mac_s), p_mac_addr)),
		// 	log_strdup(mac2string(
		// 		mac_s1, sizeof(mac_s1),
		// 		&m_beaconsInfo.single_beacon_infos[index]
		// 			 .mac_address)),
		// 	index);

		reset_beacon_info(&m_beaconsInfo.single_beacon_infos[index]);
		memcpy(&(m_beaconsInfo.single_beacon_infos[index].mac_address),
		       p_mac_addr, sizeof(*p_mac_addr));
	}

	return &m_beaconsInfo.single_beacon_infos[index];
}

bool beac_process_event(uint32_t now_ms, const bt_addr_le_t *addr,
			int8_t rssi_sample_value, adv_data_t *p_adv_data)
{
	BeaconHistory beacon_history;

	int8_t beacon_adv_rssi = signed2(p_adv_data->rssi);
	//int8_t rssi = p_adv_data->rssi;
	double m = calculateAccuracy(beacon_adv_rssi, rssi_sample_value);

	if (m > BEACON_DIST_INFINITY) {
		m = BEACON_DIST_INFINITY;
	}
	if (m < 1.0) {
		m = 1.0;
	}

	if (m > BEACON_DIST_MAX_M) {
		return false;
	}
	//LOG_INF("calculate accuyracy: %f, %u", m, (uint8_t)m);
	SingleBeaconInfo_t *pBeaconInfo =
		queue_beacon_info_head(addr, (uint8_t)m, now_ms);
	if (pBeaconInfo == NULL) {
		// This beacon measurement is ignored
		return true;
	}

	uint32_t time_diff = now_ms - pBeaconInfo->prevMeasureTime;
	if (time_diff > UINT8_MAX) {
		time_diff = UINT8_MAX;
	}
	pBeaconInfo->prevMeasureTime = now_ms;

	if (ring_buffer_is_full(&pBeaconInfo->beacon_hist_fifo)) {
		LOG_WRN("Ringbuffer is full");
		ring_buffer_dequeue(&pBeaconInfo->beacon_hist_fifo,
				    &beacon_history);
	}
	beacon_history.beacon_dist = m;
	beacon_history.time_diff = time_diff;

	ring_buffer_queue(&pBeaconInfo->beacon_hist_fifo, &beacon_history);

	char mac_s[MAC_CHARBUF_SIZE];

	LOG_INF("Device=%s  RX_RSSI=%d TX_RSSI=%d  m=%d calc=%d\n",
		log_strdup(mac2string(mac_s, sizeof(mac_s), addr)),
		(int)rssi_sample_value, (int)beacon_adv_rssi, (int)m,
		pBeaconInfo->calculated_dist);

	return true;
}

void beac_init()
{
	beac_reset();
}

void beac_reset()
{
	memset(&m_beaconsInfo, 0, sizeof(m_beaconsInfo));
	for (uint8_t i = 0; i < MAX_BEACONS; i++) {
		reset_beacon_info(&m_beaconsInfo.single_beacon_infos[i]);
	}
}
