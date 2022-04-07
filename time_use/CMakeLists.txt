list(APPEND ZEPHYR_EXTRA_MODULES
		${CMAKE_CURRENT_SOURCE_DIR}/../collar_protocol/
		)

zephyr_include_directories(
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/../../events/*
		${CMAKE_CURRENT_SOURCE_DIR}/include/
		${PROJECT_BINARY_DIR}/include/generated/)

target_sources(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/time_use.c
		${CMAKE_CURRENT_SOURCE_DIR}/../../events/ble/ble_beacon_event.c
		${CMAKE_CURRENT_SOURCE_DIR}/../../events/collar_stats/amc_events.c
		${CMAKE_CURRENT_SOURCE_DIR}/../../events/messaging/messaging_module_events.c
		${CMAKE_CURRENT_SOURCE_DIR}/../../events/movement_controller/movement_events.c
		${CMAKE_CURRENT_SOURCE_DIR}/../../events/gnss_controller/gnss_controller_events.c
		${CMAKE_CURRENT_SOURCE_DIR}/../../events/power_manager/pwr_event.c
		)

add_dependencies(${ZEPHYR_CURRENT_LIBRARY}
		collar_protocol_headers
		)