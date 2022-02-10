/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include "amc_handler.h"
#include <logging/log.h>
#include "sound_event.h"
#include "request_events.h"
#include "pasture_event.h"
#include "storage_events.h"
#include "pasture_structure.h"
#include "event_manager.h"
#include "error_event.h"

#define MODULE animal_monitor_control
LOG_MODULE_REGISTER(MODULE, CONFIG_AMC_LOG_LEVEL);

/* Cached fence data header and a max coordinate region. Links the coordinate
 * region to the header. Important! Is set to NULL everytime it's free'd
 * to ensure that we're calculating with a valid cached fence.
 */
static fence_t *cached_fence = NULL;

#define REQUEST_DATA_SEM_TIMEOUT_SEC 5
K_SEM_DEFINE(fence_data_sem, 1, 1);

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
static struct k_work_q calc_work_q;

/* Calculation work item, can be changed in future based on the algorithm
 * we use and how many functions we want to create for the distance
 * calculation algorithm.
 */
static struct k_work calc_work;

/**
 * @brief Function to request pasture on the event bus
 *        from the storage controller, in which case the
 *        storage module will memcpy its data to the passed address. This is
 *        only done when we boot up/on initialization.
 */
static void submit_request_pasture(void)
{
	struct stg_read_event *ev = new_stg_read_event();
	ev->partition = STG_PARTITION_PASTURE;
	ev->rotate = false;
	EVENT_SUBMIT(ev);
}

/**
 * @brief Work function for calculating the distance etc using fence and gnss
 *        data.
 */
void calculate_work_fn(struct k_work *item)
{
	/* Check if cached fence is valid. */
	if (cached_fence == NULL) {
		LOG_ERR("No fence data available.");
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

	/* At the moment, we just play the welcome sound everytime we get GNSS
	 * data, but this is just to test if everything is linked together.
	 * In the future, this function will contain logic that compares the
	 * cached fencedata, cached gnssdata and trigger sound events that our
	 * unit test can subscribe to.
	 */

	/** @warning The below section MUST be removed and is 
	 *           only correlated to current
	 *           unit test to see that the shell work. Its a simple
	 *           algorithm that checks if if every content of 
	 *           both fencedata and gnss data is equal to 
	 *           1337 respectively. Plays a sound the unit test
	 *           subscribes to if its correct.
	 */
	bool fencedata_correct = true;
	if (cached_fence->header.n_points <= 0) {
		fencedata_correct = false;
	}
	for (int i = 0; i < cached_fence->header.n_points; i++) {
		if (cached_fence->p_c[i].s_x_dm == 1337 &&
		    cached_fence->p_c[i].s_y_dm == 1337) {
			continue;
		}
		fencedata_correct = false;
	}

	bool gnssdata_correct = (current_gnssdata_area->lat == 1337 &&
				 current_gnssdata_area->lon == 1337);

	if (gnssdata_correct && fencedata_correct) {
		struct sound_event *s_ev = new_sound_event();
		s_ev->type = SND_WELCOME;
		EVENT_SUBMIT(s_ev);
	}

	if (!gnssdata_correct) {
		if (current_gnssdata_area->lat == 123 &&
		    current_gnssdata_area->lon == 123) {
			struct sound_event *s_ev = new_sound_event();
			s_ev->type = SND_FIND_ME;
			EVENT_SUBMIT(s_ev);
		}
	}
	/* Calculation finished, give semaphore so we can swap memory region
	 * on next GNSS request. 
	 * As well as notifying we're not using fence data area. 
	 */
	k_sem_give(&fence_data_sem);
}

void amc_module_init(void)
{
	/* Init work item and start and init calculation 
	 * work queue thread and item. 
	 */
	k_work_queue_init(&calc_work_q);
	k_work_queue_start(&calc_work_q, amc_calculation_thread_area,
			   K_THREAD_STACK_SIZEOF(amc_calculation_thread_area),
			   CONFIG_AMC_CALCULATION_PRIORITY, NULL);
	k_work_init(&calc_work, calculate_work_fn);
	submit_request_pasture();
}

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
	}

	cached_fence = k_malloc(len);

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
	if (is_pasture_ready_event(eh)) {
		submit_request_pasture();
		return false;
	}
	if (is_stg_ack_read_event(eh)) {
		struct stg_ack_read_event *ev_ack = cast_stg_ack_read_event(eh);
		if (ev_ack->partition != STG_PARTITION_PASTURE) {
			return false;
		}

		/* Update fence cache by freeing previous fence, and copying
		 * new one from storage controller.
		 */
		int err = update_pasture_cache(ev_ack->data, ev_ack->len);
		if (err) {
			char *err_msg = "Out of memory for pasture cache.";
			nf_app_fatal(ERR_SENDER_AMC, -ENOMEM, err_msg,
				     strlen(err_msg));
		}

		/* Indicate data has been consumed, so storage controller can
		 * finish up it's resources.
		 */
		struct stg_consumed_read_event *ev_consume =
			new_stg_consumed_read_event();
		EVENT_SUBMIT(ev_consume);
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
		k_work_submit_to_queue(&calc_work_q, &calc_work);
		return false;
	}
	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, ack_pasture_event);
EVENT_SUBSCRIBE(MODULE, gnssdata_event);
EVENT_SUBSCRIBE(MODULE, pasture_ready_event);
EVENT_SUBSCRIBE(MODULE, stg_ack_read_event);