/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _AMC_DIST_H_
#define _AMC_DIST_H_

#include <zephyr.h>

/** @brief Computes the distance from a point to any polygon in cached pasture.
 * 
 * @param[in] pos_x x position from gps measurement.
 * @param[in] pos_y y position from gps measurement.
 * @param[out] p_fence_index pointer to which polygon is closest.
 * @param[out] p_vertex_index pointer to which vertex is closest in closest polygon.
 * 
 * @return <0 Inside fence distance
 * @return 0 On fence line.
 * @return >0 Outside fence distance.
*/
int16_t fnc_calc_dist(int16_t pos_x, int16_t pos_y, uint8_t *p_fence_index,
		      uint8_t *p_vertex_index);

#endif /* _AMC_DIST_H_ */