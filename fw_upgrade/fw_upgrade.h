#include <zephyr.h>

/**
 * @brief Used to initilize the firmware upgrade module. This calls the DFU status event
 * and sets it to IDLE and no upgrade in progress, as well as cleanup DFU library and set 
 * bytes written to 0.
 * @return 0 on success, otherwise negative errno
 */
int fw_upgrade_module_init();