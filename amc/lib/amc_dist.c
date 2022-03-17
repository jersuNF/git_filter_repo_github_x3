/*
 * Copyright (c) 2022 Nofence AS
 */

#include <zephyr.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(amc_dist, CONFIG_AMC_LIB_LOG_LEVEL);

#include "amc_cache.h"
#include "amc_dist.h"
#include "pasture_structure.h"
#include "embedded.pb.h"

/** @brief Checks if a fence is valid with number of points and type..
 * 
 * @param fence pointer to fence(polygon) to chec.
 * 
 * @return true if valid, false if not valid.
*/
static bool fnc_valid(fence_t *fence)
{
	return (fence->m.n_points > 2 && fence->m.n_points <= 40 &&
		(fence->m.e_fence_type ==
			 FenceDefinitionMessage_FenceType_Normal ||
		 fence->m.e_fence_type ==
			 FenceDefinitionMessage_FenceType_Inverted));
	/** @todo: Also test timespan from eeprom fence definition here. */
}

/** @brief Determent if the point is inside the defined closed polyline.
 * 
 * @note Does not consider if the fence is inverted. That
 *       means, inside an inverted fence really means outside.
 * 
 * @param fence fence(polygon) to check.
 * @param testx x coordinate of test point.
 * @param testy y coordinate of test point.
 * 
 * @returns True if inside.
 */
static uint8_t fnc_PtInClosedPolyline(fence_t *fence, int16_t testx,
				      int16_t testy)
{
	uint8_t i, j;
	uint8_t c = 0;
	int32_t myX[2];
	int32_t myY[2];
	int32_t mytestx;
	int32_t mytesty;

	mytestx = testx;
	mytesty = testy;

	for (i = 0, j = (fence->m.n_points - 1); i < fence->m.n_points;
	     j = i++) {
		myX[0] = eep_FenceX(FncIndex, i);
		myX[1] = eep_FenceX(FncIndex, j);
		myY[0] = eep_FenceY(FncIndex, i);
		myY[1] = eep_FenceY(FncIndex, j);
		if (((myY[0] > mytesty) != (myY[1] > mytesty)) &&
		    (mytestx < ((myX[1] - myX[0]) * (mytesty - myY[0])) /
					       (myY[1] - myY[0]) +
				       myX[0])) {
			// Toggle each time the test results in "cutting a fenceline"
			c = !c;
		}
	}

	return c;
}

uint16_t fnc_distance(int16_t Ax, int16_t Ay, int16_t Bx, int16_t By)
{
	int32_t d1;
	int32_t d2;
	uint32_t my_distance;

	d1 = (int32_t)Ax - Bx;
	d1 *= d1;
	d2 = (int32_t)Ay - By;
	d2 *= d2;

	my_distance = (uint32_t)d1 + d2;
	my_distance = g_u32_SquareRootRounded(my_distance);

	return (uint16_t)my_distance;
}

typedef struct {
	int32_t X;
	int32_t Y;
} vec32_t;

/** @brief Compute the dot product AB x BC (314 - 330 cycles)
 * 
 * @param a AB
 * @param b BC
 * 
 * @return True if the dot product of A to B and from B to C is greater than Zero.
 *         In this case the actual position is outside the line segment, 
 *         and correct distance is from one of the endpoints of the line segment
*/
static int32_t fnc_dot(vec32_t *a, vec32_t *b)
{
	return (int32_t)a->X * b->X + (int32_t)a->Y * b->Y;
}

/** @brief Compute the distance from a segment AB to point C.
 * 
 * @param A_X x value of vertex A.
 * @param A_Y y value of vertex A.
 * @param B_X x value of vertex B.
 * @param B_Y y value of vertex B.
 * @param C_X x value of vertex C.
 * @param C_Y y value of vertex C.
 * 
 * @return The computed distance to actual vertex in polygon.
*/
static uint16_t fnc_ln_pt_dist(int16_t A_X, int16_t A_Y, int16_t B_X,
			       int16_t B_Y, int16_t C_X, int16_t C_Y)
{
	uint16_t d;
	vec32_t v, w;
	int32_t v_x, v_y;
	v.X = v_x = (int32_t)B_X - A_X;
	v.Y = v_y = (int32_t)B_Y - A_Y;
	w.X = (int32_t)C_X - A_X;
	w.Y = (int32_t)C_Y - A_Y;

	/* Check overflow. */
	if (v.X < INT16_MIN || v.X > INT16_MAX || v.Y < INT16_MIN ||
	    v.Y > INT16_MAX || w.X < INT16_MIN || w.X > INT16_MAX ||
	    w.Y < INT16_MIN || w.Y > INT16_MAX) {
		v.X /= 2;
		v.Y /= 2;
		w.X /= 2;
		w.Y /= 2;
	}

	int64_t c1 = fnc_dot(&w, &v);
	if (c1 <= 0) {
		d = fnc_distance(C_X, C_Y, A_X, A_Y);

	} else {
		int64_t c2 = fnc_dot(&v, &v);
		if (c2 <= c1) {
			d = fnc_distance(C_X, C_Y, B_X, B_Y);
		} else {
			int16_t PB_x, PB_y;
			PB_x = (int16_t)(A_X + (c1 * v_x) / c2);
			PB_y = (int16_t)(A_Y + (c1 * v_y) / c2);
			d = fnc_distance(C_X, C_Y, PB_x, PB_y);
		}
	}
	if (d > UINT16_MAX) {
		d = UINT16_MAX;
	}

	return d;
}

/** @brief Computes the distance from a point to any polygon in cached pasture.
 * 
 * @param pos_x x position from gps measurement.
 * @param pos_y y position from gps measurement.
 * @param p_fence_index pointer to which polygon is closest.
 * @param p_vertex_index pointer to which vertex is closest in closest polygon.
 * 
 * @return <0 Inside fence distance
 * @return 0 On fence line.
 * @return >0 Outside fence distance.
*/
int16_t fnc_CalcDist(int16_t pos_x, int16_t pos_y, uint8_t *p_fence_index,
		     uint8_t *p_vertex_index)
{
	/* Fetch pasture from cache. */
	pasture_t *pasture = NULL;
	get_pasture_cache(pasture);

	/* Fetch pasture info. */
	uint8_t n_fences = pasture->m.ul_total_fences;

	int16_t my_dist[FENCE_MAX];
	uint8_t vertex_index[FENCE_MAX];

	uint16_t my_tmp;
	uint8_t my_tmp_fnc_number;
	uint8_t i;

	/* Find distance to all fences. */
	for (uint8_t fence_index = 0; fence_index < n_fences; fence_index++) {
		fence_t *cur_fence = &pasture->fences[fence_index];

		my_dist[fence_index] = INT16_MAX;
		vertex_index[fence_index] = 0;

		if (fnc_valid(cur_fence)) {
			for (i = 1; i < cur_fence->m.n_points; i++) {
				my_tmp = fnc_ln_pt_dist(
					cur_fence->coordinates[i].s_x_dm,
					cur_fence->coordinates[i].s_y_dm,
					cur_fence->coordinates[i].s_x_dm - 1,
					cur_fence->coordinates[i].s_y_dm - 1,
					pos_x, pos_y);

				if (my_tmp < my_dist[fence_index]) {
					/* Save the shortest distance discovered. */
					my_dist[fence_index] = my_tmp;
					vertex_index[fence_index] = i;
				}
			}

			/* Further testing only if no overflow in earlier computing. */
			if (my_dist[fence_index] < INT16_MAX) {
				bool isInClosedPolyLine =
					fnc_PtInClosedPolyline(fence_index,
							       pos_x, pos_y);
				FenceDefinitionMessage_FenceType fenceType =
					(FenceDefinitionMessage_FenceType)
						eep_FenceType(fence_index);

				if (fenceType ==
					    FenceDefinitionMessage_FenceType_Normal &&
				    isInClosedPolyLine) {
					my_dist[fence_index] =
						-my_dist[fence_index];
				} else if (fenceType ==
						   FenceDefinitionMessage_FenceType_Inverted &&
					   !isInClosedPolyLine) {
					my_dist[fence_index] =
						-my_dist[fence_index];
				}
			}
		}
	}

	// 	Choose the fence with the shortest distance, of course prioritizing outside fences (postive) distances
	int16_t my_dist_outside_result = INT16_MAX;
	uint8_t my_dist_outside_result_ind = -1;
	int16_t my_dist_inside_result = INT16_MIN;
	uint8_t my_dist_inside_result_ind = -1;
	for (my_tmp_fnc_number = 0; my_tmp_fnc_number < n_fences;
	     my_tmp_fnc_number++) {
		if (my_dist[my_tmp_fnc_number] >
		    0) { /* Outside fence - positive. */
			if (my_dist[my_tmp_fnc_number] <
			    my_dist_outside_result) {
				my_dist_outside_result =
					my_dist[my_tmp_fnc_number];
				my_dist_outside_result_ind = my_tmp_fnc_number;
			}
		} else { /* Inside fence - negative or 0. */
			if (my_dist[my_tmp_fnc_number] >
			    my_dist_inside_result) {
				my_dist_inside_result =
					my_dist[my_tmp_fnc_number];
				my_dist_inside_result_ind = my_tmp_fnc_number;
			}
		}
	}

	if (my_dist_outside_result != INT16_MAX &&
	    my_dist_outside_result_ind != -1) {
		*p_fence_index = my_dist_outside_result_ind;
		*p_vertex_index = vertex_index[my_dist_outside_result_ind];
		return my_dist_outside_result;
	} else if (my_dist_inside_result != INT16_MIN &&
		   my_dist_inside_result_ind != -1) {
		*p_fence_index = my_dist_inside_result_ind;
		*p_vertex_index = vertex_index[my_dist_inside_result_ind];
		return my_dist_inside_result;
	} else {
		return INT16_MAX;
	}
}
