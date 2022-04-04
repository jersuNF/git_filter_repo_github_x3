/*
 * Copyright (c) 2021 Nofence AS
 */

#ifndef NOFENCE_SERVICE_H__
#define NOFENCE_SERVICE_H__

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>
#include <bluetooth/uuid.h>
#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <sys/byteorder.h>
#include <sys/printk.h>
#include <zephyr.h>
#include <zephyr/types.h>

/**
 * @brief      Attribute read frame char callback
 *
 * @param      conn The connection that is requesting to read 
 * @param      attr The attribute that’s being read 
 * @param	   buf Buffer to place the read result in 
 * @param	   len Length of data to read 
 * @param	   offset Offset to start reading from
 * @return 	   Number of bytes read, or in case of an error BT_GATT_ERR() with a specific ATT error code.
 */
ssize_t read_frame_char(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset);

/**
 * @brief      Attribute read version_char callback
 *
 * @param      conn The connection that is requesting to read 
 * @param      attr The attribute that’s being read 
 * @param	   buf Buffer to place the read result in 
 * @param	   len Length of data to read 
 * @param	   offset Offset to start reading from
 * @return 	   Number of bytes read, or in case of an error BT_GATT_ERR() with a specific ATT error code.
 */
ssize_t read_version_char(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			  void *buf, uint16_t len, uint16_t offset);

/**
 * @brief      Attribute write pwd char callback
 *
 * @param      conn The connection that is requesting to write 
 * @param      attr The attribute that’s being written  
 * @param	   buf Buffer with the data to write 
 * @param	   len Number of bytes in the buffer  
 * @param	   offset Offset to start writing from 
 * @param	   flags Flags (BT_GATT_WRITE_*)
 * @return 	   Number of bytes written, or in case of an error BT_GATT_ERR() with a specific ATT error code. 
 */
ssize_t write_pwd_char(struct bt_conn *conn, const struct bt_gatt_attr *attr,
		       const void *buf, uint16_t len, uint16_t offset,
		       uint8_t flags);

/**
 * @brief      Attribute write command char callback
 *
 * @param      conn The connection that is requesting to write 
 * @param      attr The attribute that’s being written  
 * @param	   buf Buffer with the data to write 
 * @param	   len Number of bytes in the buffer  
 * @param	   offset Offset to start writing from 
 * @param	   flags Flags (BT_GATT_WRITE_*)
 * @return 	   Number of bytes written, or in case of an error BT_GATT_ERR() with a specific ATT error code. 
 */
ssize_t write_command_char(struct bt_conn *conn,
			   const struct bt_gatt_attr *attr, const void *buf,
			   uint16_t len, uint16_t offset, uint8_t flags);

/**
 * @brief      Attribute write pb response char char callback
 *
 * @param      conn The connection that is requesting to write 
 * @param      attr The attribute that’s being written  
 * @param	   buf Buffer with the data to write 
 * @param	   len Number of bytes in the buffer  
 * @param	   offset Offset to start writing from 
 * @param	   flags Flags (BT_GATT_WRITE_*)
 * @return 	   Number of bytes written, or in case of an error BT_GATT_ERR() with a specific ATT error code. 
 */
ssize_t write_pb_response_pwd_char(struct bt_conn *conn,
				   const struct bt_gatt_attr *attr,
				   const void *buf, uint16_t len,
				   uint16_t offset, uint8_t flags);

enum command_char {
	CMD_TURN_OFF_FENCE = 0x01,
	CMD_REBOOT_AVR_MCU = 0x02,
	CMD_PLAY_SOUND = 0x03,
	CMD_DOWNLOAD_FENCE = 0x04,
	CMD_FW_INSTALL = 0x05
};

#define PB_RESPONSE_CHAR_LEN 8 /* This length must be checked */

/*
Nofence service: 
6 characteristics:
  "pwd char": 8 bytes key, write-only
  "command char": 1 byte commands, write-only
  "pb_response char": contains both FenceDefinitionResponse and FirmwareUpgradeResponse, write-only
  "version char": 4 bytes with fenceDefVersion, notify and read
  "frame char": 1 byte fence frame, notify and read

See more information on Nofence Gitlab Markup:
https://gitlab.com/nofence/nofence/-/blob/master/doc/markup/ble.md

*/
struct nofence_service_data {
	uint8_t pwd_char[8];
	uint8_t cmd[1];
	uint8_t version_char[4];
	uint8_t pb_response_char[PB_RESPONSE_CHAR_LEN];
	uint8_t frame_char[1];
};

extern struct nofence_service_data nofence_data;

#endif /* NOFENCE_SERVICE_H__ */
