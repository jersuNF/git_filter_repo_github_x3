/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _PASTURE_DEF_H_
#define _PASTURE_DEF_H_

#include <zephyr.h>
#include "embedded.pb.h"

#define FENCE_MAX 10

/** From embedded.pb.h rgulPoints_count */
#define FENCE_MAX_TOTAL_COORDINATES 40

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

		/** Fence number. */
		uint32_t fence_no;
	} m;

	fence_coordinate_t coordinates[FENCE_MAX_TOTAL_COORDINATES];
} fence_t;

#define MAX_FENCE_DEFINITION_SIZE                                                                  \
	sizeof(fence_t) + sizeof(fence_coordinate_t) * FENCE_MAX_TOTAL_COORDINATES

typedef struct {
	struct {
		uint32_t ul_fence_def_version;

		bool has_us_pasture_crc;
		uint16_t us_pasture_crc;

		bool has_keep_mode;
		bool keep_mode;

		int32_t l_origin_lat;
		int32_t l_origin_lon;

		uint32_t ul_total_fences;

		uint16_t us_k_lat;
		uint16_t us_k_lon;

		/* WIP, need to set status somewhere. */
		FenceStatus status;
	} m;

	fence_t fences[FENCE_MAX];
} pasture_t;

#endif /* _PASTURE_DEF_H_ */