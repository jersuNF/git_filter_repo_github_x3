#ifndef _NCLOGS_H_
#define _NCLOGS_H_

#include <zephyr.h>
#include "trice.h"
#include "collar_protocol.h"

/// @brief return amount of available bytes to read out
/// @return number of bytes. if negative, error.
int nclog_get_available_bytes();

/// @brief access static nclog_initialized variable
/// @return return true if nclog is initialized. false otherwise
bool nclog_is_initialized();

/// @brief Writes data to the ring buffer. When overflowing, the oldest data is discarded.
/// @param buf pointer to data to be written to buffer
/// @param len data size in bytes
/// @return 0 on success, -ENOMEM if buffer is overflowing
int nclogs_write(uint8_t *buf, size_t len);

/// @brief Reads data from ring buffer and free the space.
/// @param buf pointer to the output buffer where the data is stored. Can be NULL to only discard data.
/// @param cnt number of bytes to read from the buffer
/// @return Number of bytes written to the output buffer.
int nclogs_read(uint8_t *buf, size_t cnt);

/** @brief Read data from the ring buffer with a callback function only move the head when the callback is successful
 @param callback Function that should determine if data should be freed after read or not fn(NofenceMessage *pbuf, uint32_t).
 @param ppbuf pointer to a pointer to the Nofence message buffer passed to the callback
 @return
*/
int nclogs_read_with_callback(int (*callback)(NofenceMessage *buf, uint32_t bytes_read),
			      NofenceMessage **msg_buffer);

/// @brief Helper function for determining if a nclog is enabled
/// @param module
/// @param level Verbosity level. Lower number more verbose.
/// @return True if logging is enabled for particular module at particular level. false otherwise
bool nclog_is_enabled(eNCLOG_MODULE module, eNCLOG_LVL level);

/// @brief Set the verbosity level of a module
/// @param module
/// @param level
/// @return 0 on success -EINVAL if module or log level is invalid and request is ignored.
int nclog_set_level(eNCLOG_MODULE module, eNCLOG_LVL level);

/// @brief get log level of module
/// @param module
/// @return log level.
eNCLOG_LVL nclog_get_level(eNCLOG_MODULE module);

/// @brief Initializes the nclogs module. Ring buffer will only be initialized after a cold reboot.
void nclogs_module_init(void);

/** very verbose printing, nonsensical outside the scope of 
 * a specific feature development. CI/CD pipeline fails the build 
 * if these remain. Escalate to other severity to keep the select few.
 * @param MODULE The module which the log belongs to.
 */
#define NCLOG_DBG(MODULE, _FUNC)                                                                   \
	do {                                                                                       \
		if (nclog_is_enabled(CONCAT(eNCLOG_MODULE_, MODULE), eNCLOG_LVL_DBG)) {            \
			_FUNC;                                                                     \
		}                                                                                  \
	} while (0)

/** Provide a neutral at-a-glance state insight into what the system is doing.
 * @param MODULE The module which the log belongs to.
 */
#define NCLOG_INF(MODULE, _FUNC)                                                                   \
	do {                                                                                       \
		if (nclog_is_enabled(CONCAT(eNCLOG_MODULE_, MODULE), eNCLOG_LVL_INF)) {            \
			_FUNC;                                                                     \
		}                                                                                  \
	} while (0)

/**  Indicate that something failed to set/get/save/load due
 *  to timing/resources, core behavior is not broken.
 * @param MODULE The module which the log belongs to. 
 */
#define NCLOG_WRN(MODULE, _FUNC)                                                                   \
	do {                                                                                       \
		if (nclog_is_enabled(CONCAT(eNCLOG_MODULE_, MODULE), eNCLOG_LVL_WRN)) {            \
			_FUNC;                                                                     \
		}                                                                                  \
	} while (0)

/**  Indicate that something failed to set/get/save/load due
 *  to timing/resources, core behavior is broken and
 *  module restart may be required.
 * @param MODULE The module which the log belongs to.
 */
#define NCLOG_ERR(MODULE, _FUNC)                                                                   \
	do {                                                                                       \
		if (nclog_is_enabled(CONCAT(eNCLOG_MODULE_, MODULE), eNCLOG_LVL_ERR)) {            \
			_FUNC;                                                                     \
		}                                                                                  \
	} while (0)

/**  Indicate that something failed to set/get/save/load due
 *  to timing/resources, core behavior is broken and
 *  device restart may be required. 
 * @param MODULE The module which the log belongs to.
 */
#define NCLOG_FAT(MODULE, _FUNC)                                                                   \
	do {                                                                                       \
		if (nclog_is_enabled(CONCAT(eNCLOG_MODULE_, MODULE), eNCLOG_LVL_FAT)) {            \
			_FUNC;                                                                     \
		}                                                                                  \
	} while (0)

#define NCLOG_ALWAYS(_FUNC)                                                                        \
	do {                                                                                       \
		if (nclog_is_initialized()) {                                                      \
			_FUNC;                                                                     \
		}                                                                                  \
	} while (0)

typedef int (*logs_read_cb_fn)(NofenceMessage *, uint32_t);
#endif /* _NCLOGS_H_ */
