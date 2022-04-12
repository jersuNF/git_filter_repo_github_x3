zephyr_include_directories(
		${CMAKE_CURRENT_SOURCE_DIR}/lib/include/
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/../../events/*
		${PROJECT_BINARY_DIR}/include/generated/)

target_sources(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/lib/helpers.c
		${CMAKE_CURRENT_SOURCE_DIR}/time_use.c
		${CMAKE_CURRENT_SOURCE_DIR}/../../events/ble/ble_beacon_event.c
		${CMAKE_CURRENT_SOURCE_DIR}/../../events/messaging/messaging_module_events.c
		${CMAKE_CURRENT_SOURCE_DIR}/../../events/movement_controller/movement_events.c
		${CMAKE_CURRENT_SOURCE_DIR}/../../events/gnss_controller/gnss_controller_events.c
		${CMAKE_CURRENT_SOURCE_DIR}/../../events/power_manager/pwr_event.c
		${CMAKE_CURRENT_SOURCE_DIR}/../../events/collar_stats/histogram_events.c
		)

add_dependencies(${ZEPHYR_CURRENT_LIBRARY}
		collar_protocol_headers
		)