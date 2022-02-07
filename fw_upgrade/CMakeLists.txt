

zephyr_include_directories(
    .
    ./../../events/fw_upgrade/)

target_sources(app PRIVATE 
fw_upgrade.c
./../../events/fw_upgrade/fw_upgrade_events.c
)