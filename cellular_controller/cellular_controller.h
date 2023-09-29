#ifndef X3_FW_CELLULAR_CONTROLLER_H
#define X3_FW_CELLULAR_CONTROLLER_H

#include <zephyr.h>
#include "event_manager.h"

/**
 * @brief Used to initialize the cellular_controller.
 *        Checks that the LTE network interface and configuration are ok
 *        and that the cellular connection is up, to start the tcp
 *        connection.
 *
 * @return 0 on success, otherwise negative errno.
 */

int8_t cellular_controller_init(void);

/**
 * @brief Check whether to network connection is up. 
 *
 * @return True if ready, false otherwise.
 */
bool cellular_controller_is_ready(void);
void announce_connection_state(bool);

#ifdef CONFIG_ZTEST
void set_waiting_for_msg(bool val);
bool get_waiting_for_msg(void);
void set_connected(bool val);
bool get_connected(void);
void set_modem_is_ready(bool val);
bool get_modem_is_ready(void);
bool get_power_level_ok(void);
void set_power_level_ok(bool val);
bool get_fota_in_progress(void);
void set_fota_in_progress(bool val);
void set_switch_rat(bool val);
bool get_switch_rat(void);
void set_enable_mdm_fota(bool val);
bool get_enable_mdm_fota(void);

void set_download_complete(bool val);
bool get_download_complete(void);
void set_mdm_install_started(bool val);
bool get_mdm_install_started(void);
#endif



#endif //X3_FW_CELLULAR_CONTROLLER_H
