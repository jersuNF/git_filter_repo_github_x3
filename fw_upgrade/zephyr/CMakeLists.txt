zephyr_include_directories(.)

target_sources(app PRIVATE 
${CMAKE_CURRENT_SOURCE_DIR}/fw_upgrade.c
${CMAKE_CURRENT_SOURCE_DIR}/fw_upgrade_events.c
)