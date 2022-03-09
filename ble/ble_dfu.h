/*
 * Copyright (c) 2021 Nofence AS
 */

#ifndef _BLE_DFU_H_
#define _BLE_DFU_H_

/**
 * @brief Initialize the SMP service to do BLE FOTA 
 * 
 * @param[in] dev Runtime device. Attribute unused
 * @return 0 on success, negative error code on faliure
 */
int bt_dfu_init(void);

#endif /* _BLE_DFU_H_ */