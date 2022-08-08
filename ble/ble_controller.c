/*
 * Copyright (c) 2021 Nofence AS
 */

#include <bluetooth/bluetooth.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>
#include <bluetooth/services/nus.h>
#include <bluetooth/uuid.h>
#include <sys/ring_buffer.h>
#include <zephyr.h>
#include <zephyr/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#define MODULE ble_controller
#include <logging/log.h>

#include "ble_ctrl_event.h"
#include "ble_data_event.h"
#include "ble_conn_event.h"
#include "ble_controller.h"
#include "msg_data_event.h"

#include "nf_version.h"
#include "nf_settings.h"

#include "beacon_processor.h"
#include "ble_beacon_event.h"
#include "watchdog_event.h"
#include "error_event.h"
#if defined(CONFIG_BOARD_NF_SG25_27O_NRF52840) ||                              \
	defined(CONFIG_BOARD_NF_C25_25G_NRF52840)
#include "ble_dfu.h"
#elif CONFIG_BOARD_NATIVE_POSIX
#else
#error "Build with supported boardfile"
#endif

LOG_MODULE_REGISTER(MODULE, CONFIG_BLE_CONTROLLER_LOG_LEVEL);

static void bt_send_work_handler(struct k_work *work);

K_MEM_SLAB_DEFINE(ble_rx_slab, BLE_RX_BLOCK_SIZE, BLE_RX_BUF_COUNT,
		  BLE_SLAB_ALIGNMENT);
RING_BUF_DECLARE(ble_tx_ring_buf, BLE_TX_BUF_SIZE);

static K_SEM_DEFINE(ble_tx_sem, 0, 1);

static K_WORK_DEFINE(bt_send_work, bt_send_work_handler);

static struct bt_conn *current_conn;
static struct bt_gatt_exchange_params exchange_params;
static uint32_t nus_max_send_len;
static atomic_t atomic_bt_ready;
static atomic_t atomic_bt_adv_active;
static atomic_t atomic_bt_scan_active;
static int64_t beacon_scanner_timer;
#if CONFIG_BEACON_SCAN_ENABLE
static struct k_work_delayable periodic_beacon_scanner_work;
#endif
static struct k_work_delayable disconnect_peer_work;

static char bt_device_name[DEVICE_NAME_LEN + 1] = CONFIG_BT_DEVICE_NAME;

// Shaddow register. Should be initialized with data from EEPROM or FLASH
static uint16_t current_fw_ver = CONFIG_NOFENCE_FIRMWARE_NUMBER;
static uint32_t current_serial_number = CONFIG_NOFENCE_SERIAL_NUMBER;
static uint8_t current_battery_level;
static uint8_t current_error_flags;
static uint8_t current_collar_mode;
static uint8_t current_collar_status;
static uint8_t current_fence_status;
static uint8_t current_valid_pasture = 0x01;
static uint16_t current_fence_def_ver = 161;
static uint8_t current_hw_ver = CONFIG_NOFENCE_HARDWARE_NUMBER;
static uint16_t atmega_ver = 0xFFFF; // NB: Not in use, needed for App to work.

static uint8_t mfg_data[BLE_MFG_ARR_SIZE];

static struct bt_data ad[] = {
	[BLE_AD_IDX_FLAGS] = BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL |
							   BT_LE_AD_NO_BREDR)),
	[BLE_AD_IDX_MANUFACTURER] =
		BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data)),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
	BT_DATA(BT_DATA_NAME_COMPLETE, bt_device_name,
		(sizeof(CONFIG_BT_DEVICE_NAME) - 1)),
};

/**
 * @brief Function to fetch bluetooth mtu parameters. Will get maximum data
 * length that can be used for bt_nus_send
 *
 * @param[in] conn bluetooth connection object
 * @param[in] err Error
 * @param[in] params Pointer to GATT Exchange MTU parameters
 *
 */
static void exchange_func(struct bt_conn *conn, uint8_t err,
			  struct bt_gatt_exchange_params *params)
{
	if (!err) {
		nus_max_send_len = bt_nus_get_mtu(conn);
	}
}

/**
 * @brief Callback function called when BT connection is established
 *
 */
static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_WRN("Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", log_strdup(addr));

	current_conn = bt_conn_ref(conn);
	exchange_params.func = exchange_func;

	err = bt_gatt_exchange_mtu(current_conn, &exchange_params);
	if (err) {
		LOG_WRN("bt_gatt_exchange_mtu: %d", err);
	}

	ring_buf_reset(&ble_tx_ring_buf);

	struct ble_conn_event *event = new_ble_conn_event();
	event->conn_state = BLE_STATE_CONNECTED;
	EVENT_SUBMIT(event);
}

/**
 * @brief Callback function when bluetooth is disconnected
 *
 */
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Disconnected: %s (reason %u)", log_strdup(addr), reason);

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}

	struct ble_conn_event *event = new_ble_conn_event();
	event->conn_state = BLE_STATE_DISCONNECTED;
	EVENT_SUBMIT(event);
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

#if CONFIG_BEACON_SCAN_ENABLE
/**
 * @brief Periodic beacon scanner work function
 */
static void periodic_beacon_scanner_work_fn()
{
	/* Reschedule worker to start again after given interval */
	k_work_reschedule(&periodic_beacon_scanner_work,
			  K_SECONDS(CONFIG_BEACON_SCAN_PERIODIC_INTERVAL));
#if defined(CONFIG_WATCHDOG_ENABLE)
	/* Report alive */
	watchdog_report_module_alive(WDG_BLE_SCAN);
#endif
	/* Start scanner again if not already running */
	if (!atomic_get(&atomic_bt_scan_active)) {
		struct ble_ctrl_event *event = new_ble_ctrl_event();
		event->cmd = BLE_CTRL_SCAN_START;
		EVENT_SUBMIT(event);
	}
}
#endif
/**
 * @brief Work function to send data from rx ring buffer with bt nus
 * @param[in] work work item
 *
 */
static void bt_send_work_handler(struct k_work *work)
{
	uint16_t len;
	uint8_t *buf;
	int err;
	bool notif_disabled = false;

	do {
		len = ring_buf_get_claim(&ble_tx_ring_buf, &buf,
					 nus_max_send_len);

		err = bt_nus_send(current_conn, buf, len);
		if (err == -EINVAL) {
			notif_disabled = true;
			len = 0;
		} else if (err) {
			len = 0;
		}

		err = ring_buf_get_finish(&ble_tx_ring_buf, len);
		if (err) {
			LOG_ERR("Ring buffer size exceeds valid bytes");
			break;
		}
	} while (len != 0 && !ring_buf_is_empty(&ble_tx_ring_buf));

	if (notif_disabled) {
		/* BLE has not enabled notifications: don't accumulate data */
		ring_buf_reset(&ble_tx_ring_buf);
	}
}

/**
 * @brief Callback on bluetooth receive
 * @param[in] conn pointer to bt_conn object
 * @param[in] data pointer to data object
 * @param[in] len length of data received
 *
 */
static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
			  uint16_t len)
{
	void *buf;
	uint16_t remainder;

	remainder = len;

	do {
		uint16_t copy_len;
		int err;

		err = k_mem_slab_alloc(&ble_rx_slab, &buf, K_NO_WAIT);
		if (err) {
			LOG_WRN("BLE RX overflow");
			break;
		}

		copy_len = remainder > BLE_RX_BLOCK_SIZE ? BLE_RX_BLOCK_SIZE :
							   remainder;
		remainder -= copy_len;
		memcpy(buf, data, copy_len);

		struct ble_data_event *event = new_ble_data_event();

		event->buf = buf;
		event->len = copy_len;
		EVENT_SUBMIT(event);
	} while (remainder);
}

/**
 * @brief Callback on bluetooth send. Submit bt_send_work item to initiate
 * bt_send_work_handler
 * @param[in] conn pointer to bt_conn object
 *
 */
static void bt_sent_cb(struct bt_conn *conn)
{
	if (ring_buf_is_empty(&ble_tx_ring_buf)) {
		return;
	}

	k_work_submit(&bt_send_work);
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
	.sent = bt_sent_cb,
};

/**
 * @brief Start bluetooth advertisement with configured parameters,
 * advertise response and scan response array.
 */
static void adv_start(void)
{
	int err;

	if (!atomic_get(&atomic_bt_ready)) {
		/* Advertising will start when ready */
		LOG_WRN("Advertising not ready to start");
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE,
					      BT_GAP_ADV_SLOW_INT_MIN,
					      BT_GAP_ADV_SLOW_INT_MAX, NULL),
			      ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		char *e_msg = "Failed to start ble advertisement";
		LOG_ERR("%s (%d)", log_strdup(e_msg), err);
		nf_app_error(ERR_BLE_MODULE, err, e_msg, strlen(e_msg));
	} else {
		LOG_INF("Starting BLE advertising");
	}
}

/**
 * @brief Stop bluetooth advertisement. Set module state to standby.
 */
static void adv_stop(void)
{
	int err;

	err = bt_le_adv_stop();
	if (err) {
		char *e_msg = "Failed to stop ble advertisement";
		LOG_ERR("%s (%d)", log_strdup(e_msg), err);
		nf_app_error(ERR_BLE_MODULE, err, e_msg, strlen(e_msg));
	}
}

/**
 * @brief Function to update battery level in advertising array
 * @param[in] battery_precentage battery level
 */
static void battery_update(uint8_t battery_precentage)
{
	int err;
	current_battery_level = battery_precentage; // Update shaddow
	mfg_data[BLE_MFG_IDX_BATTERY] = current_battery_level;

	err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err && err != -EAGAIN) {
		/* Ignore error return when advertising is not running */
		LOG_WRN("bt_le_adv_update_data: %d", err);
		return;
	}
}

/**
 * @brief Function to update error flag in advertising array
 * @param[in] error_flags 0 is no erros, 1 means error available
 */
static void error_flag_update(uint8_t error_flags)
{
	int err;
	current_error_flags = error_flags; // Update shaddow
	mfg_data[BLE_MFG_IDX_ERROR] = current_error_flags;
	err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err && err != -EAGAIN) {
		/* Ignore error return when advertising is not running */
		LOG_WRN("bt_le_adv_update_data: %d", err);
		return;
	}
}

/**
 * @brief Function to update collar mode in advertising array
 * @param[in] collar_mode where 0 is normal mode, 1 is teach mode
 */
static void collar_mode_update(uint8_t collar_mode)
{
	int err;
	current_collar_mode = collar_mode; // Update shaddow
	mfg_data[BLE_MFG_IDX_COLLAR_MODE] = current_collar_mode;
	err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err && err != -EAGAIN) {
		/* Ignore error return when advertising is not running */
		LOG_WRN("bt_le_adv_update_data: %d", err);
		return;
	}
}

/**
 * @brief Function to update collar status in advertising array
 * @param[in] collar_status new status of the collar
 */
static void collar_status_update(uint8_t collar_status)
{
	int err;
	current_collar_status = collar_status; // Update shaddow
	mfg_data[BLE_MFG_IDX_COLLAR_STATUS] = current_collar_status;
	err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err && err != -EAGAIN) {
		/* Ignore error return when advertising is not running */
		LOG_WRN("bt_le_adv_update_data: %d", err);
		return;
	}
}

/**
 * @brief Function to update fence status in advertising array
 * @param[in] fence_status where 1 is fence status normal
 */
static void fence_status_update(uint8_t fence_status)
{
	int err;
	current_fence_status = fence_status; // Update shaddow
	mfg_data[BLE_MFG_IDX_FENCE_STATUS] = current_fence_status;
	err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err && err != -EAGAIN) {
		/* Ignore error return when advertising is not running */
		LOG_WRN("bt_le_adv_update_data: %d", err);
		return;
	}
}

/**
 * @brief Function to update status of valid pasture in advertising array
 * @param[in] valid_pasture where 0 is false and 1 is true
 */
static void pasture_update(uint8_t valid_pasture)
{
	int err;
	current_valid_pasture = valid_pasture; // Update shaddow
	mfg_data[BLE_MFG_IDX_VALID_PASTURE] = current_valid_pasture;
	err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err && err != -EAGAIN) {
		/* Ignore error return when advertising is not running */
		LOG_WRN("bt_le_adv_update_data: %d", err);
		return;
	}
}

/**
 * @brief Function to update fence definition version in advertising array
 * @param[in] fence_def_ver version number
 */
static void fence_def_ver_update(uint16_t fence_def_ver)
{
	int err;
	current_fence_def_ver = fence_def_ver; // Update shaddow
	mfg_data[BLE_MFG_IDX_FENCE_DEF_VER] = current_fence_def_ver;
	err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err && err != -EAGAIN) {
		/* Ignore error return when advertising is not running */
		LOG_WRN("bt_le_adv_update_data: %d", err);
		return;
	}
}
/**
 * @brief Function to initialize bt_nus and data in manufacture advertisement
 * array
 * @param[in] err error code
 */
static void bt_ready(int err)
{
	if (err) {
		LOG_ERR("%s: %d", __func__, err);
		return;
	}

	err = bt_nus_init(&nus_cb);
	if (err) {
		char *e_msg = "Bluetooth Nordic Uart init service failed";
		LOG_ERR("%s (%d)", log_strdup(e_msg), err);
		nf_app_error(ERR_BLE_MODULE, err, e_msg, strlen(e_msg));
		return;
	}

	/* Convert data to uint_8 ad array format */
	mfg_data[BLE_MFG_IDX_COMPANY_ID] =
		(NOFENCE_BLUETOOTH_SIG_COMPANY_ID & 0x00ff);
	mfg_data[BLE_MFG_IDX_COMPANY_ID + 1] =
		(NOFENCE_BLUETOOTH_SIG_COMPANY_ID & 0xff00) >> 8;
	mfg_data[BLE_MFG_IDX_NRF_FW_VER] = (current_fw_ver & 0x00ff);
	mfg_data[BLE_MFG_IDX_NRF_FW_VER + 1] = (current_fw_ver & 0xff00) >> 8;
	mfg_data[BLE_MFG_IDX_SERIAL_NR] = (current_serial_number & 0x000000ff);
	mfg_data[BLE_MFG_IDX_SERIAL_NR + 1] =
		(current_serial_number & 0x0000ff00) >> 8;
	mfg_data[BLE_MFG_IDX_SERIAL_NR + 2] =
		(current_serial_number & 0x00ff0000) >> 16;
	mfg_data[BLE_MFG_IDX_SERIAL_NR + 3] =
		(current_serial_number & 0xff000000) >> 24;
	mfg_data[BLE_MFG_IDX_BATTERY] = current_battery_level;
	mfg_data[BLE_MFG_IDX_ERROR] = current_error_flags;
	mfg_data[BLE_MFG_IDX_COLLAR_MODE] = current_collar_mode;
	mfg_data[BLE_MFG_IDX_COLLAR_STATUS] = current_collar_status;
	mfg_data[BLE_MFG_IDX_FENCE_STATUS] = current_fence_status;
	mfg_data[BLE_MFG_IDX_VALID_PASTURE] = current_valid_pasture;
	mfg_data[BLE_MFG_IDX_FENCE_DEF_VER] = (current_fence_def_ver & 0x00ff);
	mfg_data[BLE_MFG_IDX_FENCE_DEF_VER + 1] =
		(current_fence_def_ver & 0xff00) >> 8;
	mfg_data[BLE_MFG_IDX_HW_VER] = current_hw_ver;
	mfg_data[BLE_MFG_IDX_ATMEGA_VER] = (atmega_ver & 0x00ff);
	mfg_data[BLE_MFG_IDX_ATMEGA_VER + 1] = (atmega_ver & 0xff00) >> 8;

	atomic_set(&atomic_bt_ready, true);

	atomic_set(&atomic_bt_adv_active, true);

	if (atomic_get(&atomic_bt_adv_active)) {
		adv_start();
	}
}

/**
 * @brief Callback for bt data parser.
 *
 * @param data Pointer to bt_data struct
 * @param user_data Pointer to user data to return
 */
static bool data_cb(struct bt_data *data, void *user_data)
{
	adv_data_t *adv_data = user_data;
	struct net_buf_simple net_buf;
	if (data->type == BT_DATA_MANUFACTURER_DATA) {
		net_buf_simple_init_with_data(&net_buf, (void *)data->data,
					      data->data_len);
		adv_data->manuf_id = net_buf_simple_pull_be16(&net_buf);
		adv_data->beacon_dev_type = net_buf_simple_pull_u8(&net_buf);
		uint8_t data_len = net_buf_simple_pull_u8(&net_buf);
		if (data_len == BEACON_DATA_LEN) {
			memcpy(&adv_data->uuid.val,
			       net_buf_simple_pull_mem(&net_buf, 16), 16);
			adv_data->major = net_buf_simple_pull_be16(&net_buf);
			adv_data->minor = net_buf_simple_pull_be16(&net_buf);
			adv_data->rssi = net_buf_simple_pull_u8(&net_buf); //197

			LOG_DBG("Nofence beacon Major: %u Minor: %u RSSI: %u MANUF_ID: %u Beacon type: %u",
				adv_data->major, adv_data->minor,
				adv_data->rssi, adv_data->manuf_id,
				adv_data->beacon_dev_type);
		} else {
			memset(adv_data, 0, sizeof(*adv_data));
		}
		return false;
	} else {
		return true;
	}
}

/**
 * @brief Callback for reporting LE scan results.
 *
 * @param addr Advertiser LE address and type.
 * @param rssi Strength of advertiser signal.
 * @param adv_type Type of advertising response from advertiser.
 * @param buf Buffer containing advertiser data.
 */
static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
		    struct net_buf_simple *buf)
{
	int err;
	adv_data_t adv_data;
	/* Extract major_id, minor_id, tx rssi and uuid */
	bt_data_parse(buf, data_cb, (void *)&adv_data);
	if (adv_data.major == BEACON_MAJOR_ID &&
	    adv_data.minor == BEACON_MINOR_ID) {
		const uint32_t now = k_uptime_get_32();
		err = beacon_process_event(now, addr, rssi, &adv_data);
		if (err != -EIO) {
			/* Beacon found. Reset 60 seconds scan_stop countdown */
			beacon_scanner_timer = k_uptime_get();
		} else {
			char *e_msg = "Process of beacon state event error";
			LOG_ERR("%s (%d)", log_strdup(e_msg), err);
			nf_app_error(ERR_BEACON, err, e_msg, strlen(e_msg));
		}
	}

	int delta_scanner_uptime = k_uptime_get() - beacon_scanner_timer;
	if (delta_scanner_uptime > CONFIG_BEACON_SCAN_DURATION * MSEC_PER_SEC) {
		/* Beacon is not found */
		struct ble_beacon_event *bc_event = new_ble_beacon_event();
		bc_event->status = BEACON_STATUS_NOT_FOUND;
		EVENT_SUBMIT(bc_event);

		/* Stop beacon scanner. Check if scan is active */
		if (atomic_get(&atomic_bt_scan_active) == true) {
			struct ble_ctrl_event *ctrl_event =
				new_ble_ctrl_event();
			ctrl_event->cmd = BLE_CTRL_SCAN_STOP;
			EVENT_SUBMIT(ctrl_event);
		}
	}
}

static void scan_start(void)
{
	if (!atomic_get(&atomic_bt_ready)) {
		/* Scan will start when bt is ready */
		LOG_INF("Scanning not ready to start");
		return;
	}

	/* Initialize the beacon list */
	init_beacon_list();

	/* Start beacon scanner subsystem */
	struct bt_le_scan_param scan_param = {
		.type = BT_HCI_LE_SCAN_PASSIVE,
		.options = BT_LE_SCAN_OPT_NONE,
		.interval = 0x003C,
		.window = 0x0028,
	};
	int err = bt_le_scan_start(&scan_param, scan_cb);
	if (err) {
		char *e_msg = "Start Beacon scanning failed";
		LOG_ERR("%s (%d)", log_strdup(e_msg), err);
		nf_app_error(ERR_BEACON, err, e_msg, strlen(e_msg));

	} else {
		LOG_INF("Start scanning for Beacons");

		/* Start beacon scanner countdown */
		beacon_scanner_timer = k_uptime_get();
	}
}

static void scan_stop(void)
{
	int err = bt_le_scan_stop();
	if (err) {
		char *e_msg = "Stop Beacon scanning failed";
		LOG_ERR("%s (%d)", log_strdup(e_msg), err);
		nf_app_error(ERR_BLE_MODULE, err, e_msg, strlen(e_msg));
	} else {
		LOG_INF("Stop scanning for Beacons");
	}
}

static void disconnect_peer_work_fn()
{
	int err = bt_conn_disconnect(current_conn, BT_HCI_ERR_AUTH_FAIL);
	if (err) {
		LOG_ERR("Failed to disconnect paired device");
		return;
	}
	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}

	struct ble_conn_event *event = new_ble_conn_event();
	event->conn_state = BLE_STATE_DISCONNECTED;
	EVENT_SUBMIT(event);
}

int ble_module_init()
{
	uint32_t serial_id = 0;
	int err = eep_uint32_read(EEP_UID, &serial_id);
	if (err != 0) {
		char *e_msg = "Failed to read serial number from eeprom!";
		LOG_ERR("%s (%d)", log_strdup(e_msg), err);
		nf_app_error(ERR_BLE_MODULE, err, e_msg, strlen(e_msg));
	} else {
		if (serial_id > 999999) {
			strncpy(bt_device_name, "NF??????\0",
				DEVICE_NAME_LEN + 1);
		} else {
			char tmp[DEVICE_NAME_LEN + 1];
			snprintf(tmp, 7, "%i", serial_id);
			uint32_t len = strlen(tmp);
			memset(bt_device_name, '0', sizeof(bt_device_name));
			bt_device_name[0] = 'N';
			bt_device_name[1] = 'F';
			strcpy(bt_device_name + DEVICE_NAME_LEN - len, tmp);

			current_serial_number = serial_id;
		}
	}

	atomic_set(&atomic_bt_adv_active, false);
	atomic_set(&atomic_bt_scan_active, false);

	nus_max_send_len = ATT_MIN_PAYLOAD;

	/* Enable ble subsystem and start advertisement */
	err = bt_enable(bt_ready);
	if (err) {
		char *e_msg = "Failed to enable Bluetooth";
		LOG_ERR("%s (%d)", log_strdup(e_msg), err);
		nf_app_error(ERR_BLE_MODULE, err, e_msg, strlen(e_msg));
		return err;
	}

	/* Callback to monitor connected/disconnected state */
	bt_conn_cb_register(&conn_callbacks);
#if defined(CONFIG_BOARD_NF_SG25_27O_NRF52840) ||                              \
	defined(CONFIG_BOARD_NF_C25_25G_NRF52840)
	err = bt_dfu_init();
#elif CONFIG_BOARD_NATIVE_POSIX
#else
#error "Build with supported boardfile"
#endif

#if CONFIG_BEACON_SCAN_ENABLE
	/* Start scanning after beacons. Set flag to true */
	if (atomic_get(&atomic_bt_scan_active) == false) {
		scan_start();
		atomic_set(&atomic_bt_scan_active, true);
	}

	/* Init and start periodic scan work function */
	k_work_init_delayable(&periodic_beacon_scanner_work,
			      periodic_beacon_scanner_work_fn);
	k_work_reschedule(&periodic_beacon_scanner_work,
			  K_SECONDS(CONFIG_BEACON_SCAN_PERIODIC_INTERVAL));
#endif
	k_work_init_delayable(&disconnect_peer_work, disconnect_peer_work_fn);
	return 0;
}

/** 
 * @brief Event handler function
 * @param[in] eh Pointer to event handler struct
 * @return true to consume the event (event is not propagated to further
 * listners), false otherwise
 */
static bool event_handler(const struct event_header *eh)
{
	/* Send debug data */
	if (is_msg_data_event(eh)) {
		const struct msg_data_event *event = cast_msg_data_event(eh);
		if (current_conn == NULL) {
			return false;
		}

		uint32_t written =
			ring_buf_put(&ble_tx_ring_buf, event->dyndata.data,
				     event->dyndata.size);
		if (written != event->dyndata.size) {
			LOG_WRN("MSG -> BLE overflow");
		}

		uint32_t buf_utilization =
			(ring_buf_capacity_get(&ble_tx_ring_buf) -
			 ring_buf_space_get(&ble_tx_ring_buf));

		/* Simple check to start transmission. */
		/* If bt_send_work is already running, this has no effect */
		if (buf_utilization == written) {
			k_work_submit(&bt_send_work);
		}

		return false;
	}

	/* Received BLE data */
	if (is_ble_data_event(eh)) {
		const struct ble_data_event *event = cast_ble_data_event(eh);

		/* Check if memory is used */
		int num = k_mem_slab_num_used_get(&ble_rx_slab);
		if (num > 0) {
			/* All subscribers have gotten a chance to copy data at this point */
			k_mem_slab_free(&ble_rx_slab, (void *)&event->buf);
		}
		return false;
	}

	/* Received ble control event */
	if (is_ble_ctrl_event(eh)) {
		LOG_DBG("BLE CONTROL EVENT RECEIVED!");
		const struct ble_ctrl_event *event = cast_ble_ctrl_event(eh);

		switch (event->cmd) {
		case BLE_CTRL_ADV_ENABLE:
			if (!atomic_set(&atomic_bt_adv_active, true)) {
				adv_start();
			}
			break;
		case BLE_CTRL_ADV_DISABLE:
			if (atomic_set(&atomic_bt_adv_active, false)) {
				adv_stop();
			}
			break;
		case BLE_CTRL_BATTERY_UPDATE:
			battery_update(event->param.battery);
			break;
		case BLE_CTRL_ERROR_FLAG_UPDATE:
			error_flag_update(event->param.error_flags);
			break;
		case BLE_CTRL_COLLAR_MODE_UPDATE:
			collar_mode_update(event->param.collar_mode);
			break;
		case BLE_CTRL_COLLAR_STATUS_UPDATE:
			collar_status_update(event->param.collar_status);
			break;
		case BLE_CTRL_FENCE_STATUS_UPDATE:
			fence_status_update(event->param.fence_status);
			break;
		case BLE_CTRL_PASTURE_UPDATE:
			pasture_update(event->param.valid_pasture);
			break;
		case BLE_CTRL_FENCE_DEF_VER_UPDATE:
			fence_def_ver_update(event->param.fence_def_ver);
			break;
		case BLE_CTRL_SCAN_START:
			if (atomic_get(&atomic_bt_scan_active) == false) {
				scan_start();
				atomic_set(&atomic_bt_scan_active, true);
			}
			break;
		case BLE_CTRL_SCAN_STOP:
			if (atomic_get(&atomic_bt_scan_active) == true) {
				scan_stop();
				atomic_set(&atomic_bt_scan_active, false);
			}
			break;
		case BLE_CTRL_DISCONNECT_PEER:
			if (current_conn != NULL) {
				k_work_schedule(&disconnect_peer_work,
						K_MSEC(500));
			}
			break;
		default:
			/* Unhandled control message */
			__ASSERT_NO_MSG(false);
			break;
		}

		return false;
	}

	/* Reveived module state event */
	// TODO: This block could be reinitialized with a power manager module
	// if (is_module_state_event(eh)) {
	// 	const struct module_state_event *event =
	// 		cast_module_state_event(eh);

	// 	if (check_state(event, MODULE_ID(main), MODULE_STATE_READY)) {
	// 		ble_module_init();
	// 	}

	// 	return false;
	//}

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

EVENT_LISTENER(MODULE, event_handler);
EVENT_SUBSCRIBE(MODULE, ble_ctrl_event);
EVENT_SUBSCRIBE(MODULE, msg_data_event);
EVENT_SUBSCRIBE_FINAL(MODULE, ble_data_event);
