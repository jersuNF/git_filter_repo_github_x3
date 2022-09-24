#include "amc_gnss.h"
#include "amc_zone.h"
#include "amc_const.h"
#include "gnss_controller_events.h"
#include <logging/log.h>
LOG_MODULE_REGISTER(amc_gnss, CONFIG_AMC_LIB_LOG_LEVEL);

gnss_mode_t gnss_mode = GNSSMODE_NOMODE;

#define GNSSFIXBIT_FIX 0
#define GNSSFIXBIT_NUMSV 1
#define GNSSFIXBIT_DOP 2
#define GNSSFIXBIT_ACC 3
#define GNSSFIXBIT_CACC 4
#define GNSSFIXBIT_SLOPE 5
#define GNSSFIXBIT_STABILITY 6
#define GNSSFIXBIT_DISTINC 7
#define GNSSFIXBIT_DISTANCE 8
#define GNSSFIXBIT_HEIGHT 9

#define GNSSFIX_ACCEPTED_MASK                                                  \
	((1 << GNSSFIXBIT_FIX) | (1 << GNSSFIXBIT_NUMSV) |                     \
	 (1 << GNSSFIXBIT_DOP) | (1 << GNSSFIXBIT_ACC) |                       \
	 (1 << GNSSFIXBIT_HEIGHT))

#define GNSSFIX_EASY_MASK                                                      \
	((1 << GNSSFIXBIT_FIX) | (1 << GNSSFIXBIT_NUMSV) |                     \
	 (1 << GNSSFIXBIT_DOP))

#define GNSSFIX_WARN_MASK                                                      \
	((1 << GNSSFIXBIT_FIX) | (1 << GNSSFIXBIT_NUMSV) |                     \
	 (1 << GNSSFIXBIT_DOP) | (1 << GNSSFIXBIT_ACC) |                       \
	 (1 << GNSSFIXBIT_HEIGHT) | (1 << GNSSFIXBIT_CACC) |                   \
	 (1 << GNSSFIXBIT_SLOPE) | (1 << GNSSFIXBIT_STABILITY) |               \
	 (1 << GNSSFIXBIT_DISTINC) | (1 << GNSSFIXBIT_DISTANCE))

static uint32_t gnss_fix_accuracy = 0;
static uint32_t gnss_fix_accuracy_with_timeout = 0;
static struct k_mutex gnss_fix_accuracy_mutex;

static gnss_timeout_cb timeout_expired = NULL;

static void gnss_timeout_expired(struct k_timer *timer)
{
	gnss_fix_accuracy_with_timeout = 0;
	if (timeout_expired != NULL) {
		timeout_expired();
	}
}

K_TIMER_DEFINE(gnss_timeout_timer, gnss_timeout_expired, NULL);

int gnss_init(gnss_timeout_cb timeout_cb)
{
	k_mutex_init(&gnss_fix_accuracy_mutex);

	gnss_mode = GNSSMODE_NOMODE;
	gnss_fix_accuracy = 0;
	gnss_fix_accuracy_with_timeout = 0;

	timeout_expired = timeout_cb;

	return 0;
}

static int16_t get_dyn_lim_for_warn_start(int16_t dist_fence_dm,
					  uint16_t h_acc_dm)
{
	if (h_acc_dm > GNSS_HACC_UPPER_LIM_DM) {
		return INT16_MAX;
	}
	if (dist_fence_dm < LIM_WARN_MIN_DM) {
		return INT16_MAX;
	}
	//    if (distance_from_fence_dm > LIM_WARN_OFF_DM) {
	//        return INT16_MAX;
	//    }
	// See \nofence\python-scripts\gps_probability.py
	// desired_prob =  0.010000 PPF interpolated b = -1.644976

	// NOF-548: Reverted dynamic pastures,
	//int16_t new_distance_dm = LIM_WARN_MIN_DM + ((hAcc_dm * 164)/100);
	return LIM_WARN_MIN_DM;
}

int gnss_update_dist_flags(int16_t dist_avg_change, int16_t dist_change,
			   int16_t dist_incr_slope_lim, uint8_t dist_inc_count,
			   uint8_t dist_incr_count, int16_t height_delta,
			   int16_t acc_delta, int16_t mean_dist,
			   uint16_t h_acc_dm)
{
	int ret = 0;

	if (k_mutex_lock(&gnss_fix_accuracy_mutex, K_MSEC(100)) == 0) {
		if (dist_avg_change >= DIST_AVG_INCR_SLOPE_LIM) {
			gnss_fix_accuracy |= (1 << GNSSFIXBIT_SLOPE);
			gnss_fix_accuracy |= (1 << GNSSFIXBIT_DISTINC);
		} else {
			if (dist_change >= dist_incr_slope_lim) {
				gnss_fix_accuracy |= (1 << GNSSFIXBIT_SLOPE);
			}
			if (dist_inc_count >= dist_incr_count) {
				gnss_fix_accuracy |= (1 << GNSSFIXBIT_DISTINC);
			}
		}
		if ((height_delta < DELTAHEIGHT_LIM) &&
		    (acc_delta < ACCDELTA_LIM)) {
			gnss_fix_accuracy |= (1 << GNSSFIXBIT_STABILITY);
		}
		int16_t dynamic_dist =
			get_dyn_lim_for_warn_start(mean_dist, h_acc_dm);
		if (dynamic_dist != INT16_MAX && mean_dist > dynamic_dist) {
			gnss_fix_accuracy |= (1 << GNSSFIXBIT_DISTANCE);
		}

		gnss_fix_accuracy_with_timeout = gnss_fix_accuracy;

		k_mutex_unlock(&gnss_fix_accuracy_mutex);
	} else {
		ret = -EBUSY;
	}

	return ret;
}

static int gnss_check_accuracy(gnss_t *gnss_data)
{
	int ret = 0;

	if (k_mutex_lock(&gnss_fix_accuracy_mutex, K_MSEC(100)) == 0) {
		gnss_fix_accuracy = 0;
		gnss_fix_accuracy_with_timeout = 0;

		if (gnss_data->latest.pvt_flags & 1) {
			gnss_fix_accuracy |= (1 << GNSSFIXBIT_FIX);
		}
		if (gnss_data->latest.num_sv >= CONFIG_GNSS_NUMSV_LIMIT) {
			gnss_fix_accuracy |= (1 << GNSSFIXBIT_NUMSV);
		}
		if (gnss_data->latest.h_dop < CONFIG_GNSS_HDOP_LIMIT) {
			gnss_fix_accuracy |= (1 << GNSSFIXBIT_DOP);
		}
		if (gnss_data->latest.h_acc_dm < CONFIG_GNSS_HACC_LIMIT) {
			gnss_fix_accuracy |= (1 << GNSSFIXBIT_ACC);
		}
		if (gnss_data->latest.head_acc < CONFIG_GNSS_CACC_LIMIT) {
			gnss_fix_accuracy |= (1 << GNSSFIXBIT_CACC);
		}
		gnss_fix_accuracy |= (1 << GNSSFIXBIT_HEIGHT);

		gnss_fix_accuracy_with_timeout = gnss_fix_accuracy;

		k_mutex_unlock(&gnss_fix_accuracy_mutex);
	} else {
		ret = -EBUSY;
	}

	return ret;
}

int gnss_update(gnss_t *gnss_data)
{
	int ret = 0;

	/* Reset timer if we have new data */
	k_timer_start(&gnss_timeout_timer, K_MSEC(CONFIG_GNSS_TIMEOUT),
			K_NO_WAIT);

	ret = gnss_check_accuracy(gnss_data);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

int gnss_calc_xy(gnss_t *gnss_data, int16_t *x_dm, int16_t *y_dm,
		 int32_t origin_lon, int32_t origin_lat, uint16_t k_lon,
		 uint16_t k_lat)
{
	int ret = 0;

	int32_t x_dm_32, y_dm_32;

	x_dm_32 = (int32_t)(
		(k_lon * (int64_t)(gnss_data->latest.lon - origin_lon)) /
		100000);
	y_dm_32 = (int32_t)(
		(k_lat * (int64_t)(gnss_data->latest.lat - origin_lat)) /
		100000);

	if ((x_dm_32 > (int32_t)INT16_MAX) || (y_dm_32 > (int32_t)INT16_MAX) ||
	    (x_dm_32 < (int32_t)INT16_MIN) || (y_dm_32 < (int32_t)INT16_MIN)) {
		/* Position is not updated if overflow */
		ret = -EOVERFLOW;
	} else {
		*x_dm = (int16_t)x_dm_32;
		*y_dm = (int16_t)y_dm_32;
	}

	return ret;
}

int gnss_update_mode(gnss_mode_t mode)
{
	if (mode >= GNSSMODE_SIZE) {
		return -EINVAL;
	}

	gnss_mode = mode;
	struct gnss_set_mode_event *ev = new_gnss_set_mode_event();
	ev->mode = gnss_mode;
	EVENT_SUBMIT(ev);

	return 0;
}

gnss_mode_t gnss_get_mode(void)
{
	return gnss_mode;
}

bool gnss_has_fix(void)
{
	bool has_fix = false;

	if (k_mutex_lock(&gnss_fix_accuracy_mutex, K_MSEC(100)) == 0) {
		has_fix = ((gnss_fix_accuracy & (1 << GNSSFIXBIT_FIX)) ==
			   (1 << GNSSFIXBIT_FIX));

		k_mutex_unlock(&gnss_fix_accuracy_mutex);
	}

	return has_fix;
}

bool gnss_has_accepted_fix(void)
{
	bool has_accepted_fix = false;

	if (k_mutex_lock(&gnss_fix_accuracy_mutex, K_MSEC(100)) == 0) {
		has_accepted_fix =
			((gnss_fix_accuracy & GNSSFIX_ACCEPTED_MASK) ==
			 GNSSFIX_ACCEPTED_MASK);

		k_mutex_unlock(&gnss_fix_accuracy_mutex);
	}

	return has_accepted_fix;
}

bool gnss_has_easy_fix(void)
{
	bool has_easy_fix = false;

	if (k_mutex_lock(&gnss_fix_accuracy_mutex, K_MSEC(100)) == 0) {
		has_easy_fix = ((gnss_fix_accuracy_with_timeout &
				 GNSSFIX_EASY_MASK) == GNSSFIX_EASY_MASK);

		k_mutex_unlock(&gnss_fix_accuracy_mutex);
	}

	return has_easy_fix;
}

bool gnss_has_warn_fix(void)
{
	bool has_warn_fix = false;

	if (k_mutex_lock(&gnss_fix_accuracy_mutex, K_MSEC(100)) == 0) {
		has_warn_fix = ((gnss_fix_accuracy & GNSSFIX_WARN_MASK) ==
				GNSSFIX_WARN_MASK);

		k_mutex_unlock(&gnss_fix_accuracy_mutex);
	}

	return has_warn_fix;
}
