/*
 * Copyright (c) 2021 Nofence AS
 */

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

/* Cached fence data header and a max coordinate region. Links the coordinate
 * region to the header. Important! Is set to NULL everytime it's free'd
 * to ensure that we're calculating with a valid cached fence.
 */
static fence_t *cached_fence = NULL;
static size_t cached_fence_size = 0;
atomic_t new_fence_version = ATOMIC_INIT(0);

#define REQUEST_DATA_SEM_TIMEOUT_SEC 5
K_SEM_DEFINE(fence_data_sem, 1, 1);
static inline int update_pasture_cache(uint8_t *data, size_t len);

/* Use two memory regions so we can swap the pointer between them
 * so that instead of waiting for semaphore to be released, we schedule
 * a pointer swap to the other region once we've read the data. Use an
 * atomic variable if both threads try to access and update that
 * gnss data is available or consumed.
 */
static gnss_struct_t cached_gnssdata_area_1;
static gnss_struct_t cached_gnssdata_area_2;
static gnss_struct_t *current_gnssdata_area = &cached_gnssdata_area_1;
atomic_t new_gnss_written = ATOMIC_INIT(false);

/* Thread stack area that we use for the calculation process. We add a work
 * item here when we have data available from GNSS.
 * This thread can then use the calculation function algorithm to
 * determine what it needs to do in regards to sound and zap events that
 * this calculation function can submit to event handler.
 */
K_THREAD_STACK_DEFINE(amc_calculation_thread_area, CONFIG_AMC_CALCULATION_SIZE);
static struct k_work_q amc_work_q;

/* Calculation work item, can be changed in future based on the algorithm
 * we use and how many functions we want to create for the distance
 * calculation algorithm.
 */
static struct k_work calc_work;

static struct k_work process_new_fence_work;

static inline int update_pasture_cache(uint8_t *data, size_t len)
{
	int err = k_sem_take(&fence_data_sem,
			     K_SECONDS(REQUEST_DATA_SEM_TIMEOUT_SEC));
	if (err) {
		LOG_ERR("Error semaphore, retry request pasture here?");
		return err;
	}

	/* Free previous fence if any. */
	if (cached_fence != NULL) {
		k_free(cached_fence);
		cached_fence = NULL;
		cached_fence_size = 0;
	}

	cached_fence = k_malloc(len);
	cached_fence_size = len;

	if (cached_fence == NULL) {
		LOG_ERR("No memory left for caching the fence.");
		k_sem_give(&fence_data_sem);
		return -ENOMEM;
	}
	/* Memcpy the flash contents. */
	memcpy(cached_fence, data, len);

	/* Can add calculation check if fence is valid if we want here. */
	LOG_INF("Updated pasture cache to US_ID: %d",
		cached_fence->header.us_id);

	k_sem_give(&fence_data_sem);
	return 0;
}

static inline int update_pasture_from_stg(void)
{
	int err = stg_read_pasture_data(update_pasture_cache);
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
	int err = update_pasture_from_stg();
	if (err) {
		new_fence_version = 0;
		LOG_ERR("Error caching new pasture from storage controller.");
		return;
	}

	/* Submit event that we have now began to use the new fence. */
	struct update_fence_version *ver = new_update_fence_version();
	ver->fence_version = (uint32_t)atomic_get(&new_fence_version);
	EVENT_SUBMIT(ver);
}

/**
 * @brief Work function for calculating the distance etc using fence and gnss
 *        data.
 */
void calculate_work_fn(struct k_work *item)
{
	/* Check if cached fence is valid. */
	if (cached_fence == NULL) {
		char *err_msg = "No fence data available during calculation.";
		nf_app_fatal(ERR_SENDER_AMC, -ENODATA, err_msg,
			     strlen(err_msg));
		return;
	}

	int err = k_sem_take(&fence_data_sem,
			     K_SECONDS(REQUEST_DATA_SEM_TIMEOUT_SEC));
	if (err) {
		LOG_ERR("Error waiting for fence data semaphore to release.");
		return;
	}

	/* Check if we received new data, swap the pointer to point
	 * at the newly written GNSS data area if we have.
	 */
	if (atomic_get(&new_gnss_written)) {
		if (current_gnssdata_area == &cached_gnssdata_area_1) {
			current_gnssdata_area = &cached_gnssdata_area_2;
		} else {
			current_gnssdata_area = &cached_gnssdata_area_1;
		}
		atomic_set(&new_gnss_written, false);
	}

	/* Calculations here, REMOVE BELOW CODE. */
	if (cached_fence->p_c[0].s_x_dm == 0xDE) {
		if (current_gnssdata_area->lat == 1337) {
			struct sound_event *ev = new_sound_event();
			ev->type = SND_WELCOME;
			EVENT_SUBMIT(ev);
		}
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
	k_work_init(&calc_work, calculate_work_fn);
	k_work_init(&process_new_fence_work, process_new_fence_fn);

	/* Fetch the fence from external flash. */
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
		struct new_fence_available *event =
			cast_new_fence_available(eh);
		atomic_set(&new_fence_version, event->fence_version);
		k_work_submit_to_queue(&amc_work_q, &process_new_fence_work);
		return false;
	}
	if (is_gnssdata_event(eh)) {
		struct gnssdata_event *event = cast_gnssdata_event(eh);

		/* Copy GNSS data struct to area not being read from
		 * and indicate that we have new data available. So the
		 * calculation thread can swap pointers.
		 */
		if (current_gnssdata_area == &cached_gnssdata_area_1) {
			memcpy(&cached_gnssdata_area_2, &event->gnss,
			       sizeof(gnss_struct_t));
		} else {
			memcpy(&cached_gnssdata_area_1, &event->gnss,
			       sizeof(gnss_struct_t));
		}

		atomic_set(&new_gnss_written, true);

		/* Call the calculation thread. */
		k_work_submit_to_queue(&amc_work_q, &calc_work);
		return false;
	}
	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, gnssdata_event);
EVENT_SUBSCRIBE(MODULE, new_fence_available);