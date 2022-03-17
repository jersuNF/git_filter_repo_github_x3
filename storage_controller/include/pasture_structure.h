/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _PASTURE_DEF_H_
#define _PASTURE_DEF_H_

#include <zephyr.h>

#define FENCE_MAX 10
#define FENCE_MAX_TOTAL_COORDINATES 300

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
	union {
		/** Fence ID. */
		uint16_t us_id;

		/** Number of coordinates in polygon. */
		uint8_t n_points;

		/** Fence type. */
		uint8_t e_fence_type;

		/** Fence number. */
		uint32_t fence_no;
	} m;

	/** Coordinates. An example of how to dynamically allocate 
	 *  fence coordinates is found at tests/storage_controller and
	 *  storage_helper.c and look for get_simulated_fence_data()
	*/
	fence_coordinate_t coordinates[];
} fence_t;

typedef struct {
	union {
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
	} m;

	/* Uses same dynamic allocation as it's coordinates. */
	fence_t fences[];
} pasture_t;

#define FENCE_MAX_DEFINITION_SIZE                                              \
	FENCE_MAX * sizeof(fence_t) +                                          \
		(FENCE_MAX_TOTAL_COORDINATES * sizeof(fence_coordinate_t))

/* 

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

typedef struct {
	int16_t s_x_dm;
	int16_t s_y_dm;
} fence_coordinate_t;

typedef struct {
	union {
		uint16_t us_id;
		uint8_t n_points;
		uint8_t e_fence_type;
		uint32_t fence_no;
	} m;
	fence_coordinate_t coordinates[];
} fence_t;

typedef struct {
	union {
		uint32_t ul_fence_def_version;

		uint8_t has_us_pasture_crc;
		uint16_t us_pasture_crc;

		uint8_t has_keep_mode;
		uint8_t keep_mode;

		int32_t l_origin_lat;
		int32_t l_origin_lon;

		uint32_t ul_total_fences;

		uint16_t us_k_lat;
		uint16_t us_k_lon;
	} m;
	fence_t fences[];
} pasture_t;


static int total_fences_size = 0;

int main()
{
    int fences = 5;
    pasture_t* pasture;
    
    for(int f = 0; f < fences; f++) {
        
        int fence_pts = f;
        int fence_size = sizeof(fence_t) + (sizeof(fence_coordinate_t) * fence_pts);
        
        total_fences_size += fence_size;
        
        if (pasture != NULL) {
            free(pasture);
        }
        
        //Risk of data on pasture pointer location is overwritten since we
        //free'd the pasture. The pointer is kept, so data on heap is kept
        //until malloc again, unless another thread called malloc in between.
        //
        
        pasture = (pasture_t*)malloc(sizeof(pasture_t) + total_fences_size);
        
        for(int i = 0; i < fence_pts; i++) {
            pasture->fences[f].coordinates[i].s_x_dm = i;
            pasture->fences[f].coordinates[i].s_y_dm = i;
        }
    }
    
    
    for(int f = 0; f < fences; f++) {
        int fence_pts = f;
        for(int i = 0; i < fence_pts; i++) {
            printf("Fence %i, coordinate %i [%i, %i]\n", f, i, pasture->fences[f].coordinates[i].s_x_dm, pasture->fences[f].coordinates[i].s_y_dm);
        }
    }
    
    
    if (pasture == NULL) {
        return 0;
    }
    
    free(pasture);
    return 0;
}


*/

#endif /* _PASTURE_DEF_H_ */