/*
 * Copyright (c) 2021 Nofence AS
 */

#ifndef _MESSAGING_H_
#define _MESSAGING_H_

#include <zephyr.h>
#include "event_manager.h"

extern struct k_sem ble_ctrl_sem;
extern struct k_sem ble_data_sem;
extern struct k_sem lte_proto_sem;

#endif /* _MESSAGING_H_ */