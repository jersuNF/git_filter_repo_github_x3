/*
 * Copyright (c) 2021 Nofence AS
 */

#ifndef _BUZZER_H_
#define _BUZZER_H_

#include <zephyr.h>
#include "event_manager.h"
#include "sound_event.h"

int buzzer_module_init(void);
void play_type(enum sound_event_type type);

#endif /* _BUZZER_H_ */