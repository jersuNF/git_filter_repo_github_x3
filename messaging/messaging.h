/*
 * Copyright (c) 2021 Nofence AS
 */

#ifndef _MESSAGING_H_
#define _MESSAGING_H_

#include <zephyr.h>
#include "event_manager.h"
#include "collar_protocol.h"
#include "embedded.pb.h"

typedef struct {
	Mode collar_mode;
	CollarStatus collar_status;
	FenceStatus fence_status;
	uint32_t fence_version;
	uint16_t flash_erase_count;
	uint16_t zap_count;
} collar_state_struct_t;

typedef void (*fota_wdt_cb)(void);

/**
 * @brief Initialization of the messaging module. 
 * @return Returns 0 if successfull, otherwise negative error code.
 */
int messaging_module_init(void);

/**
 * @brief Register a cb called by fota_wdt_trigger
 * @param callback to register
 */
void fota_wdt_cb_register(fota_wdt_cb wdt_cb);

/**
 * @brief Trigger registered watchdog callback
 */
void fota_wdt_trigger();
#endif /* _MESSAGING_H_ */
