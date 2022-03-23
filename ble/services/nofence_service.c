/*
 * Copyright (c) 2021 Nofence AS
 */

#include "nofence_service.h"

#include <bluetooth/bluetooth.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>
#include <bluetooth/uuid.h>
#include <errno.h>
#include <power/reboot.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <sys/byteorder.h>
#include <sys/crc.h>
#include <sys/printk.h>
#include <zephyr.h>
#include <zephyr/types.h>

#include "ble_cmd_event.h"
#include "ble_ctrl_event.h"

#define MODULE nofence_ble_service
#include <logging/log.h>
LOG_MODULE_REGISTER(MODULE, CONFIG_BLE_SERVICE_LOG_LEVEL);

// Filled with dummy data for test
struct nofence_service_data nofence_data = {
	.cmd = { 0 },
	.frame_char = { 0 },
	.pb_response_char = { 0, 0, 0, 0, 0, 0, 0, 0 },
	.pwd_char = { 0, 0, 0, 0, 0, 0, 0, 0 },
	.version_char = { 0 }
};

static struct bt_uuid_128 bt_uuid_nofence_service =
	BT_UUID_INIT_128(0xe1, 0x2c, 0xf3, 0xe9, 0xa7, 0xfd, 0x36, 0xb7, 0xe8,
			 0x45, 0x92, 0x98, 0xcd, 0xab, 0x85, 0x8c);

static struct bt_uuid_128 bt_uuid_find_me_char =
	BT_UUID_INIT_128(0xe1, 0x2c, 0xf3, 0xe9, 0xa7, 0xfd, 0x36, 0xb7, 0xe8,
			 0x45, 0x92, 0x98, 0xef, 0xbe, 0x85, 0x8c);
static struct bt_uuid_128 bt_uuid_pwd_char =
	BT_UUID_INIT_128(0xe1, 0x2c, 0xf3, 0xe9, 0xa7, 0xfd, 0x36, 0xb7, 0xe8,
			 0x45, 0x92, 0x98, 0x5e, 0xba, 0x85, 0x8c);

static struct bt_uuid_128 bt_uuid_command_char =
	BT_UUID_INIT_128(0xe1, 0x2c, 0xf3, 0xe9, 0xa7, 0xfd, 0x36, 0xb7, 0xe8,
			 0x45, 0x92, 0x98, 0xad, 0xde, 0x85, 0x8c);

static struct bt_uuid_128 bt_uuid_pb_response_char =
	BT_UUID_INIT_128(0xe1, 0x2c, 0xf3, 0xe9, 0xa7, 0xfd, 0x36, 0xb7, 0xe8,
			 0x45, 0x92, 0x98, 0x02, 0x00, 0x85, 0x8c);

static struct bt_uuid_128 bt_uuid_version_char =
	BT_UUID_INIT_128(0xe1, 0x2c, 0xf3, 0xe9, 0xa7, 0xfd, 0x36, 0xb7, 0xe8,
			 0x45, 0x92, 0x98, 0x03, 0x00, 0x85, 0x8c);

static struct bt_uuid_128 bt_uuid_frame_char =
	BT_UUID_INIT_128(0xe1, 0x2c, 0xf3, 0xe9, 0xa7, 0xfd, 0x36, 0xb7, 0xe8,
			 0x45, 0x92, 0x98, 0x04, 0x00, 0x85, 0x8c);

ssize_t read_version_char(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			  void *buf, uint16_t len, uint16_t offset)
{
	LOG_INF("Read version char: 0x%x:0x%x:0x%x:0x%x",
		nofence_data.version_char[0], nofence_data.version_char[1],
		nofence_data.version_char[2], nofence_data.version_char[3]);

	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &nofence_data.version_char,
				 sizeof(nofence_data.version_char));
}

ssize_t read_frame_char(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	LOG_INF("Read frame char: 0x%x", nofence_data.frame_char[0]);

	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &nofence_data.frame_char,
				 sizeof(nofence_data.frame_char));
}

ssize_t write_pb_response_char(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr, const void *buf,
			       uint16_t len, uint16_t offset, uint8_t flags)
{
	LOG_INF("Received write request of PB response char of size %d", len);
	return len;
}

/* NB: This is deprecated, but kept here for testing with the current nofence app */
ssize_t write_find_me_char(struct bt_conn *conn,
			   const struct bt_gatt_attr *attr, const void *buf,
			   uint16_t len, uint16_t offset, uint8_t flags)
{
	LOG_INF("Received find me char of len %d", len);
	struct ble_cmd_event *ev = new_ble_cmd_event();
	ev->cmd = CMD_PLAY_SOUND;
	EVENT_SUBMIT(ev);
	return len;
}

ssize_t write_command_char(struct bt_conn *conn,
			   const struct bt_gatt_attr *attr, const void *buf,
			   uint16_t len, uint16_t offset, uint8_t flags)
{
	if (len == sizeof(nofence_data.cmd)) {
		uint8_t ble_key_ret[8];
		memset(ble_key_ret, 0, sizeof(ble_key_ret));
		int ret =
			eep_read_ble_sec_key(ble_key_ret, sizeof(ble_key_ret));
		if (ret < 0) {
			LOG_ERR("Failed to read ble_sec_key, err %d", ret);
			return len;
		}

		LOG_HEXDUMP_INF(ble_key_ret, 8, "BLE sec key");
		ret = memcmp(nofence_data.pwd_char, ble_key_ret, 8);
		if (ret == 0) {
			/* BLE sec key match. Set nofence_data.cmd shaddow register*/
			uint8_t *cmd_char = (uint8_t *)buf;
			memcpy(nofence_data.cmd, cmd_char, len);
			LOG_INF("Received write request of command char 0x%x",
				nofence_data.cmd[0]);

			/* Submit event to messaging module. */
			struct ble_cmd_event *ev = new_ble_cmd_event();
			ev->cmd = nofence_data.cmd[0];
			EVENT_SUBMIT(ev);
		} else {
			LOG_ERR("BLE sec key mismatch. Remember to write correct key to PWD char first");
			/* TODO: Disconnect peer here */

			struct ble_ctrl_event *ctrl_event =
				new_ble_ctrl_event();
			ctrl_event->cmd = BLE_CTRL_DISCONNECT_PEER;
			EVENT_SUBMIT(ctrl_event);
			return len;
		}
	} else {
		LOG_ERR("Size %d of written command char is wrong. Expect %d bytes",
			len, sizeof(nofence_data.cmd));
	}
	return len;
}

ssize_t write_pwd_char(struct bt_conn *conn, const struct bt_gatt_attr *attr,
		       const void *buf, uint16_t len, uint16_t offset,
		       uint8_t flags)
{
	if (len == sizeof(nofence_data.pwd_char)) {
		// Grab and copy the pwd char
		uint8_t *pwd_char = (uint8_t *)buf;
		memcpy(nofence_data.pwd_char, pwd_char,
		       sizeof(nofence_data.pwd_char));

		LOG_INF("Write pwd char request with data: 0x%x-%x-%x-%x-%x-%x-%x-%x",
			nofence_data.pwd_char[0], nofence_data.pwd_char[1],
			nofence_data.pwd_char[2], nofence_data.pwd_char[3],
			nofence_data.pwd_char[4], nofence_data.pwd_char[5],
			nofence_data.pwd_char[6], nofence_data.pwd_char[7]);
	} else {
		LOG_ERR("Size %d of written pwd char is too long", len);
	}
	return len;
}

static void version_char_ccc_cfg_changed(const struct bt_gatt_attr *attr,
					 uint16_t value)
{
	ARG_UNUSED(attr);

	bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);

	LOG_INF("Version char notifications %s",
		notif_enabled ? "enabled" : "disabled");
}

static void frame_char_ccc_cfg_changed(const struct bt_gatt_attr *attr,
				       uint16_t value)
{
	ARG_UNUSED(attr);

	bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);

	LOG_INF("Frame char notifications %s",
		notif_enabled ? "enabled" : "disabled");
}

BT_GATT_SERVICE_DEFINE(
	nofence_service, BT_GATT_PRIMARY_SERVICE(&bt_uuid_nofence_service),
	/* FIND ME CHAR */
	BT_GATT_CHARACTERISTIC(&bt_uuid_find_me_char.uuid, BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_WRITE, NULL, write_find_me_char,
			       NULL),

	/* PWD CHAR */
	BT_GATT_CHARACTERISTIC(&bt_uuid_pwd_char.uuid, BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_WRITE, NULL, write_pwd_char, NULL),

	/* COMMAND CHAR */
	BT_GATT_CHARACTERISTIC(&bt_uuid_command_char.uuid, BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_WRITE, NULL, write_command_char,
			       NULL),

	/* PB RESPONSE CHAR */
	BT_GATT_CHARACTERISTIC(&bt_uuid_pb_response_char.uuid,
			       BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL,
			       write_pb_response_char, NULL),
	/* VERSION CHAR */
	BT_GATT_CHARACTERISTIC(&bt_uuid_version_char.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, read_version_char, NULL,
			       NULL),
	BT_GATT_CCC(version_char_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	/* Frame char */
	BT_GATT_CHARACTERISTIC(&bt_uuid_frame_char.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, read_frame_char, NULL, NULL),
	BT_GATT_CCC(frame_char_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));
