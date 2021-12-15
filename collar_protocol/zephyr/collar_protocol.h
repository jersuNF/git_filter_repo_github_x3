/*
 * Copyright (c) 2021 Nofence AS
 */

#include "embedded.pb.h"

/**
 * @brief Decodes a binary blob of protobuf data into
 *        a NofenceMessage data structure.
 *
 * @param[in] src_data Pointer to source data to decode.
 * @param[in] src_data_size Number of bytes of data in src_data.
 * @param[out] dst_msg Pointer to NofenceMessage structure to populate
 *                     with parsed contents.
 *
 * @return 0 on success, otherwise negative error code.
 */
int collar_protocol_decode(uint8_t *src_data, size_t src_data_size, 
			NofenceMessage *dst_msg);

/**
 * @brief Encodes a NofenceMessage data structure into
 *        a binary blob of protobuf data.
 *
 * @param[in] src_msg Pointer to NofenceMessage structure to encode.
 * @param[out] dst_data Pointer to destination data buffer.
 * @param[in] dst_data_max_size Maximum number of bytes in destination data
 *                              buffer.
 * @param[out] dst_data_size Pointer to size variable for number of
 *                           bytes written to dst_data.
 *
 * @return 0 on success, otherwise negative error code.
 */
int collar_protocol_encode(NofenceMessage *src_msg, uint8_t *dst_data,
			size_t dst_data_max_size, size_t *dst_data_size);