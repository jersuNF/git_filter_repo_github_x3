

zephyr_include_directories(
    .
    ./../../events/power_manager/
    ./../../events/fw_upgrade/
    ./../../events/error_handler/)

target_sources(app PRIVATE 
fw_upgrade.c
./../../events/fw_upgrade/fw_upgrade_events.c
./../../events/error_handler/error_event.c
./../../events/power_manager/pwr_event.c
)