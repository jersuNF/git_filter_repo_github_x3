/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _LOG_DEF_H_
#define _LOG_DEF_H_

#include <zephyr.h>
#include "embedded.pb.h"

typedef struct {
	SequenceMessage seq_1;
	SequenceMessage_2 seq_2;
} log_rec_t;

#endif /* _LOG_DEF_H_ */