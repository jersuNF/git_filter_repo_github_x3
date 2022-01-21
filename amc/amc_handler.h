/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _AMC_HANDLER_H_
#define _AMC_HANDLER_H_

#include <zephyr.h>

/** @brief Struct containing GPS data. */
typedef struct {
	int32_t lat;
	int32_t lon;

	/** Corrected position.*/
	int16_t x;
	/** Corrected position.*/
	int16_t y;

	/** Set if overflow because of too far away from origin position.*/
	uint8_t overflow;

	/** Height above ellipsoid [dm].*/
	int16_t height;

	/** 2-D speed [cm/s]*/
	uint16_t speed;

	/** Movement direction (-18000 to 18000 Hundred-deg).*/
	int16_t head_veh;

	/** Horizontal dilution of precision.*/
	uint16_t h_dop;

	/** Horizontal Accuracy Estimate [DM].*/
	uint16_t h_acc_dm;

	/** Vertical Accuracy Estimate [DM].*/
	uint16_t v_acc_dm;

	/** Heading accuracy estimate [Hundred-deg].*/
	uint16_t head_acc;

	/** Number of SVs used in Nav Solution.*/
	uint8_t num_sv;

	/** UBX-NAV-PVT flags as copied.*/
	uint8_t pvt_flags;

	/** UBX-NAV-PVT valid flags as copies.*/
	uint8_t pvt_valid;

	/** Milliseconds since position report from UBX.*/
	uint16_t age;

	/** UBX-NAV-SOL milliseconds since receiver start or reset.*/
	uint32_t msss;

	/** UBX-NAV-SOL milliseconds since First Fix.*/
	uint32_t ttff;
} gnss_struct_t;

/** @brief See gps_struct_t for descriptions. */
typedef struct {
	int32_t lat;
	int32_t lon;
	uint32_t unix_timestamp;
	uint16_t h_acc_d;
	int16_t head_veh;
	uint16_t head_acc;
	uint8_t pvt_flags;
	uint8_t num_sv;
	uint16_t hdop;
	int16_t height;
	int16_t baro_height;
	uint32_t msss;
	uint8_t gps_mode;
} gnss_last_fix_struct_t;

#define GNSS_DEFINITION_SIZE (FENCE_MAX * sizeof(gps_struct_t))

/** @todo PSH, the structure is aligned to 32 bit unsigned 
 * as returned from server. However, y values are placed
 * before X-values. Look into this. Currently, we must put Y before X.
 */
typedef struct {
	/** Relative coordinated of fence pole 
         *  in DECIMETERS from global origin.
         */

	/** NORTHING. */
	int16_t s_x_dm;

	/** EASTING. */
	int16_t s_y_dm;
} fence_coordinate_t;

typedef struct {
	/** Fence ID. */
	uint16_t us_id;

	/** Number of coordinates in polygon. */
	uint8_t n_points;

	/** Fece type. */
	uint8_t e_fence_type;

	/** Direct pointer to fence coordinate start. */
	fence_coordinate_t *p_c; //
} fence_header_t;

#define FENCE_MAX 10
#define FENCE_MAX_TOTAL_COORDINATES 300
#define FENCE_MAX_DEFINITION_SIZE                                              \
	(FENCE_MAX * sizeof(fence_header_t) +                                  \
	 FENCE_MAX_TOTAL_COORDINATES * sizeof(fence_coordinate_t))

/** @brief Initializes the work calculation thread and puts
 *         request events for fence and GNSS data.
 */
void amc_module_init(void);

/**
 * @brief Function to request fence data on the event bus, in which case the
 *        storage module will memcpy its data to the passed address. This is
 *        only done when we boot up/on initialization, and on user updates
 *        in which case speed does not matter, as well as not
 *        needing the continuity as the GNSS data does.
 */
void submit_request_fencedata(void);

/**
 * @brief Function to request GNSS data on the event bus, in which case the
 *        GNSS module will memcpy its data to the passed address pointer.
 *        Up for discussion because it might be too slow, and perhaps could
 *        be better solutions for this continous/periodic data transfer.
 */
void submit_request_gnssdata(void);

#endif /* _AMC_HANDLER_H_ */