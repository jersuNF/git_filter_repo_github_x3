/*
 * Copyright (c) 2018-2019 Peter Bigot Consulting, LLC
 * Copyright (c) 2019-2020 Nordic Semiconductor ASA
 * Copyright (c) 2021 Nofence AS
 * 
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APPLICATION_BATTERY_H_
#define APPLICATION_BATTERY_H_

/** 
 * @brief Measure the battery voltage.
 * @return the battery voltage in millivolts, or a negative error
 * code.
 */
int battery_sample(void);

/** 
 * @brief A point in a battery discharge curve sequence.
 *
 * A discharge curve is defined as a sequence of these points, where
 * the first point has #lvl_pptt set to 10000 and the last point has
 * #lvl_pptt set to zero.  Both #lvl_pptt and #lvl_mV should be
 * monotonic decreasing within the sequence.
 */
struct battery_level_point {
	/** Remaining life at #lvl_mV. */
	uint16_t lvl_pptt;

	/** Battery voltage at #lvl_pptt remaining life. */
	uint16_t lvl_mV;
};

/** 
 * @brief Calculate the estimated battery level based on a measured voltage.
 * @param batt_mV measured battery voltage level.
 * @param curve the discharge curve for the type of battery installed
 * on the system.
 * @return the estimated remaining capacity in precentage.
 */
unsigned int battery_level_soc(unsigned int batt_mV,
			       const struct battery_level_point *curve);

/** 
 * @brief Calculate a moving average of the battery voltage
 * @return averaged voltage in mV, or negative error code on error
 */
int battery_sample_averaged(void);

/** 
 * @brief Moving average struct copied from old code 
 */
typedef struct {
	uint32_t total;
	uint16_t average;
	uint16_t N; // working number of samples
	uint16_t MAX_SAMPLES;
} MovAvg;

/** 
 * @brief Calculate the estimated battery level based on a measured voltage.
 * @param p Pointer to MovAvg struct
 * @param val value to add
 * @return Moving average
 */
uint16_t approx_moving_average(MovAvg *p, uint16_t val);

/** 
 * @brief Set up the battery divider
 * @return 0 on success, otherwise -ENOENT error code if device not ready.
 */
int battery_setup(void);

#endif /* APPLICATION_BATTERY_H_ */
