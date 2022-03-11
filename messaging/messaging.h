/*
 * Copyright (c) 2021 Nofence AS
 */

#ifndef _MESSAGING_H_
#define _MESSAGING_H_

#include <zephyr.h>
#include "event_manager.h"
#include "collar_protocol.h"

extern struct k_sem ble_ctrl_sem;
extern struct k_sem ble_data_sem;
extern struct k_sem lte_proto_sem;

typedef struct {
	Mode collar_mode;
	CollarStatus collar_status;
	FenceStatus fence_status;
	uint32_t fence_version;
	uint16_t flash_erase_count;
	uint16_t zap_count;

} collar_state_struct_t;

/**
 * @brief Used to initilize the messaging module. 
 *        Mostly inits the delayable work poller for modem checks.
 * 
 * @return 0 on success, otherwise negative errno.
 */
void messaging_module_init(void);
#endif /* _MESSAGING_H_ */