/*
 * Copyright (c) 2021 Nofence AS
 */

#include <zephyr.h>
#include <pb_decode.h>
#include <pb_encode.h>

#include "collar_protocol.h"

#if NofenceMessage_size > CONFIG_NOFENCE_MESSAGE_EXPECTED_MAX_SIZE
#error "Somebody has changed the embedded.proto file so that the maximum \
	message size is inacceptable - could crash client"
#endif

int collar_protocol_decode(uint8_t *src_data, size_t src_data_size, NofenceMessage *dst_msg)
{
	/* Check that input parameters are valid. */
	if ((src_data == NULL) || (src_data_size == 0) || (dst_msg == NULL)) {
		return -EINVAL;
	}

	memset(dst_msg, 0, sizeof(NofenceMessage));

	/* Decoding a message requires a stream buffer to be defined, which
	 * uses source data as the input of the stream. 
	 */
	pb_istream_t stream = pb_istream_from_buffer(src_data, src_data_size);
	if (!pb_decode(&stream, NofenceMessage_fields, dst_msg)) {
		return -EILSEQ;
	}

	return 0;
}

int collar_protocol_encode(NofenceMessage *src_msg, uint8_t *dst_data, size_t dst_data_max_size,
			   size_t *dst_data_size)
{
	/* Check that input parameters are valid. */
	if ((src_msg == NULL) || (dst_data == NULL) || (dst_data_max_size == 0) ||
	    (dst_data_size == NULL)) {
		return -EINVAL;
	}

	/* Encoding a message requires a stream buffer to be defined, which
	 * uses destination data as the output of the stream. 
	 */
	pb_ostream_t stream = pb_ostream_from_buffer(dst_data, dst_data_max_size);
	if (!pb_encode(&stream, NofenceMessage_fields, src_msg)) {
		return -EMSGSIZE;
	}

	*dst_data_size = stream.bytes_written;

	return 0;
}
