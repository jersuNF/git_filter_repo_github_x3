/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _ANO_DEF_H_
#define _ANO_DEF_H_

#include <zephyr.h>
#include "UBX.h"

/**
 * This structure defines all messages stored in the external flash.
 *
 * @param header Header of entry in flash
 * @param buf Actual data, size is header.len bytes
 *
 */
typedef struct {
	/**
	 * @brief ANO id corresponds to the day the ANO packet was generated on the server
	 */
	uint16_t ano_id;

	/** @brief, the sequence number of this ANO packet for the corresponding day */
	uint16_t sequence_id;

	/* @brief The actual full u-blox ANO packet, including header and checksum */
	UBX_MGA_ANO_RAW_t raw_ano;
} ano_rec_t;

#endif /* _ANO_DEF_H_ */