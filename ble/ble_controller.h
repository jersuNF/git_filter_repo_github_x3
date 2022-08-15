/*
 * Copyright (c) 2021 Nofence AS
 */

#ifndef _BLE_CONTROLLER_H_
#define _BLE_CONTROLLER_H_

#define BLE_RX_BLOCK_SIZE (CONFIG_BT_L2CAP_TX_MTU - 3)
#define BLE_RX_BUF_COUNT 4
#define BLE_SLAB_ALIGNMENT 4

#define BLE_TX_BUF_SIZE (CONFIG_MSG_BUF_SIZE * 2)

#define BLE_AD_IDX_FLAGS 0
#define BLE_AD_IDX_MANUFACTURER 1

#define BLE_MFG_IDX_COMPANY_ID 0
#define BLE_MFG_IDX_NRF_FW_VER 2
#define BLE_MFG_IDX_SERIAL_NR 4
#define BLE_MFG_IDX_BATTERY 8
#define BLE_MFG_IDX_ERROR 9
#define BLE_MFG_IDX_COLLAR_MODE 10
#define BLE_MFG_IDX_COLLAR_STATUS 11
#define BLE_MFG_IDX_FENCE_STATUS 12
#define BLE_MFG_IDX_VALID_PASTURE 13
#define BLE_MFG_IDX_FENCE_DEF_VER 14
#define BLE_MFG_IDX_HW_VER 16
#define BLE_MFG_IDX_ATMEGA_VER 17

#define BLE_MFG_ARR_SIZE 19
#define NOFENCE_BLUETOOTH_SIG_COMPANY_ID 0x05AB

#define ATT_MIN_PAYLOAD 20 /* Minimum L2CAP MTU minus ATT header */

#define DEVICE_NAME_LEN 8 /* NF999999   */
/**
 * @brief Used to initilize the ble module
 * 
 * @return 0 on success, otherwise negative errno.
 */
int ble_module_init();

#endif /*_BLE_CONTROLLER_H_ */