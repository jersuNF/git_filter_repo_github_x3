/*
 * Copyright (c) 2022 Nofence AS
 */

#ifndef _DIAGNOSTICS_TYPES_H_
#define _DIAGNOSTICS_TYPES_H_

/** @brief Enumeration used for identifying the interface for receiving and 
 *         sending diagnostics data. 
 */
enum diagnostics_interface {
	DIAGNOSTICS_ALL = 0,
	DIAGNOSTICS_RTT,
	DIAGNOSTICS_BLE,
	DIAGNOSTICS_NONE
};

#endif /* _DIAGNOSTICS_TYPES_H_ */