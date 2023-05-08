/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _SEQ_DEF_H_
#define _SEQ_DEF_H_

#include <zephyr.h>
#include "embedded.pb.h"

typedef struct {
	SequenceMessage seq_1;
	SequenceMessage_2 seq_2;
} seq_rec_t;

#endif /* _SEQ_DEF_H_ */