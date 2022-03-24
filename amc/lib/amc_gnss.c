#include "amc_gnss.h"
#include "amc_zone.h"

gnss_mode_t gnss_mode = GNSSMODE_NOMODE;

#define GNSSFIXBIT_FIX        0
#define GNSSFIXBIT_NUMSV      1
#define GNSSFIXBIT_DOP        2
#define GNSSFIXBIT_ACC        3
#define GNSSFIXBIT_CACC       4
#define GNSSFIXBIT_SLOPE      5
#define GNSSFIXBIT_STABILITY  6
#define GNSSFIXBIT_DISTINC    7
#define GNSSFIXBIT_DISTANCE   8
#define GNSSFIXBIT_HEIGHT     9

#define GNSSFIX_ACCEPTED_MASK ((1<<GNSSFIXBIT_FIX)|\
			       (1<<GNSSFIXBIT_NUMSV)|\
			       (1<<GNSSFIXBIT_DOP)|\
			       (1<<GNSSFIXBIT_ACC)|\
			       (1<<GNSSFIXBIT_HEIGHT))

#define GNSSFIX_EASY_MASK     ((1<<GNSSFIXBIT_FIX)|\
			       (1<<GNSSFIXBIT_NUMSV)|\
			       (1<<GNSSFIXBIT_DOP))

#define GNSSFIX_WARN_MASK     ((1<<GNSSFIXBIT_FIX)|\
			       (1<<GNSSFIXBIT_NUMSV)|\
			       (1<<GNSSFIXBIT_DOP)|\
			       (1<<GNSSFIXBIT_ACC)|\
			       (1<<GNSSFIXBIT_HEIGHT)|\
			       (1<<GNSSFIXBIT_CACC)|\
			       (1<<GNSSFIXBIT_SLOPE)|\
			       (1<<GNSSFIXBIT_STABILITY)|\
			       (1<<GNSSFIXBIT_DISTINC)|\
			       (1<<GNSSFIXBIT_DISTANCE))

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

static int gnss_check_accuracy(gnss_t* gnss_data)
{
	int ret = 0;

	if (k_mutex_lock(&gnss_fix_accuracy_mutex, K_MSEC(100)) == 0) {
		gnss_fix_accuracy = 0;
		gnss_fix_accuracy_with_timeout = 0;

		if (gnss_data->latest.pvt_flags & 1) {
			gnss_fix_accuracy |= (1<<GNSSFIXBIT_FIX);
		}
		if (gnss_data->latest.num_sv >= CONFIG_GNSS_NUMSV_LIMIT) {
			gnss_fix_accuracy |= (1<<GNSSFIXBIT_NUMSV);
		}
		if (gnss_data->latest.h_dop < CONFIG_GNSS_HDOP_LIMIT) {
			gnss_fix_accuracy |= (1<<GNSSFIXBIT_DOP);
		}
		if (gnss_data->latest.h_acc_dm < CONFIG_GNSS_HACC_LIMIT) {
			gnss_fix_accuracy |= (1<<GNSSFIXBIT_ACC);
		}
		if (gnss_data->latest.head_acc < CONFIG_GNSS_CACC_LIMIT) {
			gnss_fix_accuracy |= (1<<GNSSFIXBIT_CACC);
		}
		gnss_fix_accuracy |= (1<<GNSSFIXBIT_HEIGHT);
		
		gnss_fix_accuracy_with_timeout = gnss_fix_accuracy;

		k_mutex_unlock(&gnss_fix_accuracy_mutex);
	} else {
		ret = -EBUSY;
	}

	return ret;
}

int gnss_update(gnss_t* gnss_data)
{
	int ret = 0;

	/* Reset timer if we have valid fix */
	if (gnss_data->fix_ok) {
		k_timer_start(&gnss_timeout_timer, 
			K_MSEC(CONFIG_GNSS_TIMEOUT), 
			K_NO_WAIT);
	}
	
	ret = gnss_check_accuracy(gnss_data);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

int gnss_set_mode(gnss_mode_t mode)
{
	if (mode >= GNSSMODE_SIZE) {
		return -EINVAL;
	}

	gnss_mode = mode;

	/** @todo Send gnss mode through controller when implemented */
	
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
		has_fix = ((gnss_fix_accuracy&(1<<GNSSFIXBIT_FIX)) == \
			   (1<<GNSSFIXBIT_FIX));
		
		k_mutex_unlock(&gnss_fix_accuracy_mutex);
	}

	return has_fix;
}

bool gnss_has_accepted_fix(void)
{
	bool has_accepted_fix = false;

	if (k_mutex_lock(&gnss_fix_accuracy_mutex, K_MSEC(100)) == 0) {
		has_accepted_fix = 
			((gnss_fix_accuracy&GNSSFIX_ACCEPTED_MASK) == \
			 GNSSFIX_ACCEPTED_MASK);
		
		k_mutex_unlock(&gnss_fix_accuracy_mutex);
	}
	
	return has_accepted_fix;
}

bool gnss_has_easy_fix(void)
{
	bool has_easy_fix = false;

	if (k_mutex_lock(&gnss_fix_accuracy_mutex, K_MSEC(100)) == 0) {
		has_easy_fix = 
		    ((gnss_fix_accuracy_with_timeout&GNSSFIX_EASY_MASK) == \
		     GNSSFIX_EASY_MASK);
		
		k_mutex_unlock(&gnss_fix_accuracy_mutex);
	}
	
	return has_easy_fix;
}

/** @todo Either remove or add setters for remaining flags */
bool gnss_has_warn_fix(void)
{
	bool has_warn_fix = false;

	if (k_mutex_lock(&gnss_fix_accuracy_mutex, K_MSEC(100)) == 0) {
		has_warn_fix = ((gnss_fix_accuracy&GNSSFIX_WARN_MASK) == \
				GNSSFIX_WARN_MASK);
		
		k_mutex_unlock(&gnss_fix_accuracy_mutex);
	}
	
	return has_warn_fix;
}