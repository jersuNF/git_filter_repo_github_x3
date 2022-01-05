/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include "messaging.h"
#include <logging/log.h>

#include "collar_protocol.h"

#define LOG_MODULE_NAME messaging
LOG_MODULE_REGISTER(LOG_MODULE_NAME, CONFIG_MESSAGING_LOG_LEVEL);

#define MSG_SIMULATED_THREAD_STACK_SIZE 800
#define MSG_SIMULATED_THREAD_SLEEP 1000
#define MSG_SIMULATED_THREAD_PRIORITY 1

/* Thread for handling various messages on that appear on the messsage queue. */
static K_THREAD_STACK_DEFINE(msg_simulated_thread_stack,
			     MSG_SIMULATED_THREAD_STACK_SIZE);

static struct k_thread msg_simulated_thread;

#define MSG_BUF_SIZE CONFIG_MSG_BUF_SIZE

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
	int err;

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(MODULE, event_handler);
