/*
 * Copyright (c) 2021 Nofence AS
 */
#include "amc_cache.h"
#include "amc_dist.h"
#include "amc_zone.h"

#include <zephyr.h>
#include "amc_handler.h"
#include <logging/log.h>
#include "sound_event.h"
#include "request_events.h"
#include "pasture_structure.h"
#include "event_manager.h"
#include "error_event.h"
#include "messaging_module_events.h"

#include "storage.h"

#define MODULE animal_monitor_control
LOG_MODULE_REGISTER(MODULE, CONFIG_AMC_LOG_LEVEL);

/* Thread stack area that we use for the calculation process. We add a work
 * item here when we have data available from GNSS.
 * This thread can then use the calculation function algorithm to
 * determine what it needs to do in regards to sound and zap events that
 * this calculation function can submit to event handler.
 */
K_THREAD_STACK_DEFINE(amc_calculation_thread_area, CONFIG_AMC_CALCULATION_SIZE);

static struct k_work_q amc_work_q;
static struct k_work process_new_gnss_work;
static struct k_work process_new_fence_work;

static inline int update_pasture_from_stg(void)
{
	int err = stg_read_pasture_data(set_pasture_cache);
	if (err == -ENODATA) {
		char *err_msg = "No pasture found on external flash.";
		nf_app_warning(ERR_SENDER_AMC, err, err_msg, strlen(err_msg));
		return 0;
	} else if (err) {
		char *err_msg = "Couldn't update pasture cache in AMC.";
		nf_app_fatal(ERR_SENDER_AMC, err, err_msg, strlen(err_msg));
		return err;
	}
	return 0;
}

void process_new_fence_fn(struct k_work *item)
{
	/* Update AMC cache. */
	int cache_ret = update_pasture_from_stg();

	/* Take pasture sem, since we need to access the version to send
	 * to messaging module.
	 */
	int err = k_sem_take(&fence_data_sem,
			     K_SECONDS(CONFIG_FENCE_CACHE_TIMEOUT_SEC));
	if (err) {
		LOG_ERR("Error taking pasture semaphore for version check.");
		return;
	}

	pasture_t *pasture = NULL;
	get_pasture_cache(pasture);

	if (cache_ret) {
		pasture->m.ul_fence_def_version = 0;
		LOG_ERR("Error caching new pasture from storage controller.");
		return;
	}

	/* Submit event that we have now began to use the new fence. */
	struct update_fence_version *ver = new_update_fence_version();
	ver->fence_version = pasture->m.ul_fence_def_version;
	EVENT_SUBMIT(ver);

	k_sem_give(&fence_data_sem);
}

/**
 * @brief Work function for processing new gnss data that have just been stored.
 */
void process_new_gnss_data_fn(struct k_work *item)
{
	/* Take fence semaphore since we're going to use the cached area. */
	int err = k_sem_take(&fence_data_sem,
			     K_SECONDS(CONFIG_FENCE_CACHE_TIMEOUT_SEC));
	if (err) {
		LOG_ERR("Error waiting for fence data semaphore to release.");
		return;
	}

	/* Fetch cached fence and size. */
	pasture_t *pasture = NULL;

	err = get_pasture_cache(pasture);
	if (err || pasture == NULL) {
		char *msg = "Error getting fence cache.";
		nf_app_fatal(ERR_SENDER_AMC, err, msg, strlen(msg));
		return;
	}

	/* Fetch new, cached gnss data. */
	gnss_struct_t *gnss = NULL;
	err = get_gnss_cache(gnss);
	if (err || gnss == NULL) {
		char *msg = "Error getting gnss cahce.";
		nf_app_fatal(ERR_SENDER_AMC, err, msg, strlen(msg));
		return;
	}

	/* Calculation finished, give semaphore so we can swap memory region
	 * on next GNSS request. 
	 * As well as notifying we're not using fence data area. 
	 */
	k_sem_give(&fence_data_sem);
}

int amc_module_init(void)
{
	/* Init work item and start and init calculation 
	 * work queue thread and item. 
	 */
	k_work_queue_init(&amc_work_q);
	k_work_queue_start(&amc_work_q, amc_calculation_thread_area,
			   K_THREAD_STACK_SIZEOF(amc_calculation_thread_area),
			   CONFIG_AMC_CALCULATION_PRIORITY, NULL);
	k_work_init(&process_new_gnss_work, process_new_gnss_data_fn);
	k_work_init(&process_new_fence_work, process_new_fence_fn);

	/* Fetch the fence from external flash and update fence cache. */
	return update_pasture_from_stg();
}

/**
 * @brief Main event handler function. 
 * 
 * @param[in] eh Event_header for the if-chain to 
 *               use to recognize which event triggered.
 * 
 * @return True or false based on if we want to consume the event or not.
 */
static bool event_handler(const struct event_header *eh)
{
	if (is_new_fence_available(eh)) {
		k_work_submit_to_queue(&amc_work_q, &process_new_fence_work);
		return false;
	}
	if (is_gnssdata_event(eh)) {
		struct gnssdata_event *event = cast_gnssdata_event(eh);

		int err = set_gnss_cache(&event->gnss);
		if (err) {
			char *msg = "Could not set gnss cahce.";
			nf_app_error(ERR_SENDER_AMC, err, msg, strlen(msg));
			return false;
		}

		k_work_submit_to_queue(&amc_work_q, &process_new_gnss_work);
		return false;
	}
	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, gnssdata_event);
EVENT_SUBSCRIBE(MODULE, new_fence_available);
