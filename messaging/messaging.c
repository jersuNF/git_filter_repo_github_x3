/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include "messaging.h"
#include <logging/log.h>
#include "ble_data_event.h"
#include "collar_protocol.h"
#include "http_downloader.h"

#define MODULE messaging
LOG_MODULE_REGISTER(MODULE, CONFIG_MESSAGING_LOG_LEVEL);

/* 4 means 4-byte alignment. */
K_MSGQ_DEFINE(messaging_msgq, sizeof(struct ble_data_event),
	      CONFIG_MSGQ_BLE_DATA_SIZE, 4);

static atomic_t messaging_thread_active;
static K_SEM_DEFINE(messaging_thread_sem, 0, 1);

struct k_poll_event events[1] = { K_POLL_EVENT_STATIC_INITIALIZER(
	K_POLL_TYPE_FIFO_DATA_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY,
	&messaging_msgq, 0) };

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

void messaging_thread_fn()
{
}

K_THREAD_DEFINE(messaging_thread, CONFIG_MESSAGING_THREAD_SIZE,
		messaging_thread_fn, NULL, NULL, NULL,
		CONFIG_MESSAGING_THREAD_PRIORITY, 0, 0);
