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

/* Cached data arrays for seeing position and fence data to determine which
   sounds should be played.
 */
static uint8_t cached_fencedata[CONFIG_STATIC_FENCEDATA_SIZE];
static uint8_t cached_gnssdata[CONFIG_STATIC_GNSSDATA_SIZE];

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
	struct request_fencedata_event *req_fd_e =
		new_request_fencedata_event();

	req_fd_e->data = cached_fencedata;
	req_fd_e->len = CONFIG_STATIC_FENCEDATA_SIZE;
	EVENT_SUBMIT(req_fd_e);
}

/**
 * @brief Function to request GNSS data on the event bus, in which case the
 *        GNSS module will memcpy its data to the passed address.
 *        Up for discussion because it might be too slow, and perhaps could
 *        be better solutions for this continous/periodic data transfer.
 */
void submit_request_gnssdata(void)
{
	struct request_gnssdata_event *req_gd_e = new_request_gnssdata_event();

	req_gd_e->data = cached_gnssdata;
	req_gd_e->len = CONFIG_STATIC_GNSSDATA_SIZE;

	EVENT_SUBMIT(req_gd_e);
}

/**
 * @brief Work function for calculating the distance etc using fence and gnss
 *        data.
 */
void calculate_work_fn(struct k_work *item)
{
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
	 *           both fencedata and gnss data is equal and 
	 *           0xDE or 0xAE respectively. Plays a sound the unit test
	 *           subscribes to if its correct.
	 */
	bool fencedata_correct = true;
	for (int i = 0; i < CONFIG_STATIC_FENCEDATA_SIZE; i++) {
		if (cached_fencedata[i] == 0xDE ||
		    cached_fencedata[i] == 0xAD) {
			continue;
		}
		fencedata_correct = false;
	}

	bool gnssdata_correct = true;
	for (int i = 0; i < CONFIG_STATIC_GNSSDATA_SIZE; i++) {
		if (cached_gnssdata[i] == 0xDE || cached_gnssdata[i] == 0xAD) {
			continue;
		}
		gnssdata_correct = false;
	}

	if (fencedata_correct && gnssdata_correct) {
		struct sound_event *s_ev = new_sound_event();
		s_ev->type = SND_WELCOME;
		EVENT_SUBMIT(s_ev);
	}
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
		}
		return false;
	}
	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, amc_ack_event);