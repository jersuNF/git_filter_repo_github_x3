#include "amc_gnss.h"

static gnss_timeout_cb timeout_expired = NULL;

static void gnss_timeout_expired(struct k_timer *timer)
{
	if (timeout_expired != NULL) {
		timeout_expired();
	}
}

K_TIMER_DEFINE(gnss_timeout_timer, gnss_timeout_expired, NULL);

int gnss_init(gnss_timeout_cb timeout_cb)
{
	timeout_expired = timeout_cb;

	return 0;
}

int gnss_validate_and_update(gnss_t* gnss_data)
{
	if (gnss_data->fix_ok) {
		
		k_timer_start(&gnss_timeout_timer, 
			K_MSEC(CONFIG_GNSS_TIMEOUT), 
			K_NO_WAIT);
	}
	
	return 0;
}