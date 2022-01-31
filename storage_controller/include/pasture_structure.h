/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _PASTURE_DEF_H_
#define _PASTURE_DEF_H_

#include <zephyr.h>

#define FENCE_MAX 10
#define FENCE_MAX_TOTAL_COORDINATES 300

/** @todo Currently, we must put Y before X.
 */
typedef struct {
	/** Relative coordinates of fence pole 
         *  in DECIMETERS from global origin.
         */

	/** NORTHING. */
	int16_t s_x_dm;

	/** EASTING. */
	int16_t s_y_dm;
} fence_coordinate_t;

typedef struct {
	struct {
		/** Fence ID. */
		uint16_t us_id;

		/** Number of coordinates in polygon. */
		uint8_t n_points;

		/** Fence type. */
		uint8_t e_fence_type;
	} header;

	/** Coordinates. */
	fence_coordinate_t p_c[FENCE_MAX_TOTAL_COORDINATES];
} fence_t;

#define FENCE_MAX_DEFINITION_SIZE FENCE_MAX * sizeof(fence_t)

#endif /* _PASTURE_DEF_H_ */