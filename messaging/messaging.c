/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include "messaging.h"
#include <logging/log.h>

#include "collar_protocol.h"

#define MODULE messaging
LOG_MODULE_REGISTER(MODULE, CONFIG_MESSAGING_LOG_LEVEL);

#define MESSAGING_THREAD_STACK_SIZE 800
#define MESSAGING_THREAD_SLEEP 1000
#define MESSAGING_THREAD_PRIORITY 1
//
//K_MSGQ_DEFINE(messaging_msgq, sizeof(struct dfu_fragment_event),
//	      CONFIG_MSGQ_FRAGMENT_SIZE, 4);
//
//static atomic_t messaging_thread_active;
//static K_SEM_DEFINE(messaging_thread_sem, 0, 1);

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
	//int err;

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(MODULE, event_handler);

//
//void apply_fragment_thread_fn()
//{
//}
//
//K_THREAD_DEFINE(apply_fragment_thread, CONFIG_MSGQ_APPLY_FRAGMENT_THREAD_SIZE,
//		apply_fragment_thread_fn, NULL, NULL, NULL,
//		K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
