/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include "amc_handler.h"
#include <logging/log.h>
#include "sound_event.h"
#include "request_events.h"
#include "event_manager.h"

#define MODULE animal_monitor_control
LOG_MODULE_REGISTER(MODULE, CONFIG_AMC_LOG_LEVEL);

/* Cached fence data header and a max coordinate region. Links the coordinate
 * region to the header.
 */
static fence_coordinate_t cached_fence_coordinates[FENCE_MAX_TOTAL_COORDINATES];
static fence_header_t cached_fence_header = { .p_c = cached_fence_coordinates };

K_SEM_DEFINE(fence_data_sem, 1, 1);

/* Use two memory regions so we can swap the pointer between them
 * so that instead of waiting for semaphore to be released, we schedule
 * a pointer swap to the other region once we've read the data.
 */
static gnss_struct_t cached_gnssdata_area_1;
static gnss_struct_t cached_gnssdata_area_2;
static gnss_struct_t *current_gnssdata_area = &cached_gnssdata_area_1;

/* Only swap the pointer in the request function
 * if we're currently not doing a calculation.
 */
K_SEM_DEFINE(gnss_data_sem, 1, 1);
#define REQUEST_DATA_SEM_TIMEOUT_SEC 5

/* Thread stack area that we use for the calculation process. We add a work
 * item here when we have data available from GNSS as well as fencedata.
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
 * @brief Function to request fence data on the event bus, in which case the
 *        storage module will memcpy its data to the passed address. This is
 *        only done when we boot up/on initialization, and on user updates
 *        in which case speed does not matter, as well as not
 *        needing the continuity as the GNSS data does.
 */
void submit_request_fencedata(void)
{
	int err = k_sem_take(&fence_data_sem,
			     K_SECONDS(REQUEST_DATA_SEM_TIMEOUT_SEC));
	if (err) {
		LOG_ERR("Error waiting fence data semaphore to release.");
	}

	struct request_fencedata_event *req_fd_e =
		new_request_fencedata_event();
	req_fd_e->fence = &cached_fence_header;
	EVENT_SUBMIT(req_fd_e);
}

/**
 * @brief Function to request GNSS data on the event bus, in which case the
 *        GNSS module will memcpy its data to the passed address.
 */
void submit_request_gnssdata(void)
{
	/* Always write to the one that we're currently not reading from. 
	 * We check the semaphore so we only request if we're not
	 * doing a calculation and if we have done one calculation since the
	 * previous request since it's free'd in the calculation function.
	 */
	int err = k_sem_take(&gnss_data_sem,
			     K_SECONDS(REQUEST_DATA_SEM_TIMEOUT_SEC));
	if (err) {
		LOG_ERR("Error waiting for calculation to finish.");
	}

	struct request_gnssdata_event *req_gd_e = new_request_gnssdata_event();
	if (current_gnssdata_area == &cached_gnssdata_area_1) {
		req_gd_e->gnss = &cached_gnssdata_area_2;
	} else {
		req_gd_e->gnss = &cached_gnssdata_area_1;
	}

	EVENT_SUBMIT(req_gd_e);
}

/**
 * @brief Work function for calculating the distance etc using fence and gnss
 *        data.
 */
void calculate_work_fn(struct k_work *item)
{
	int err = k_sem_take(&fence_data_sem,
			     K_SECONDS(REQUEST_DATA_SEM_TIMEOUT_SEC));
	if (err) {
		LOG_ERR("Error waiting fence data semaphore to release.");
	}

	/* We received new data, swap the pointer to point
	 * at the newly written GNSS data area. This means that subsequent
	 * calls to request_gnss_data function will pass a pointer to
	 * the area not in use by this function.
	 */
	if (current_gnssdata_area == &cached_gnssdata_area_1) {
		current_gnssdata_area = &cached_gnssdata_area_2;
	} else {
		current_gnssdata_area = &cached_gnssdata_area_1;
	}

	LOG_INF("Compares fencedata and gnss data here.");

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
	if (cached_fence_header.n_points <= 0) {
		fencedata_correct = false;
	}
	for (int i = 0; i < cached_fence_header.n_points; i++) {
		if (cached_fence_header.p_c[i].s_x_dm == 1337 &&
		    cached_fence_header.p_c[i].s_y_dm == 1337) {
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
	k_sem_give(&gnss_data_sem);
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

	submit_request_fencedata();

	/* We can assume that this function will be called periodically, 
	 * in which case we need to discuss the best solution due to delay
	 * on event bus, as well as tedious request/write/read/ack.
	 */
	submit_request_gnssdata();
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
	if (is_amc_ack_event(eh)) {
		struct amc_ack_event *event = cast_amc_ack_event(eh);

		/* If we received GNSS data, start the calculating function. */
		if (event->type == AMC_REQ_GNSSDATA) {
			k_work_submit_to_queue(&calc_work_q, &calc_work);
		} else if (event->type == AMC_REQ_FENCEDATA) {
			k_sem_give(&fence_data_sem);
		}
		return false;
	}
	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, amc_ack_event);