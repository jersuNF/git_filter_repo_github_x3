list(APPEND ZEPHYR_EXTRA_MODULES
        ${CMAKE_CURRENT_SOURCE_DIR}/../collar_protocol/
        )

zephyr_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../fw_upgrade/http_download/
    ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/crc/
    ${CMAKE_CURRENT_SOURCE_DIR}/../../events/ble/
    ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/ble/services
    ${CMAKE_CURRENT_SOURCE_DIR}/../../events/lte/
	${CMAKE_CURRENT_SOURCE_DIR}/../../events/cellular_controller/
	${CMAKE_CURRENT_SOURCE_DIR}/../../events/gps_controller/
	${CMAKE_CURRENT_SOURCE_DIR}/../../events/messaging/
	${CMAKE_CURRENT_SOURCE_DIR}/../../events/buzzer/
	${CMAKE_CURRENT_SOURCE_DIR}/../../events/error_handler/
	${CMAKE_CURRENT_SOURCE_DIR}/../../events/storage/
	${CMAKE_CURRENT_SOURCE_DIR}/../../modules/storage_controller/
	${CMAKE_CURRENT_SOURCE_DIR}/include/
	${PROJECT_BINARY_DIR}/include/generated/)

target_sources(app PRIVATE 
${CMAKE_CURRENT_SOURCE_DIR}/messaging.c
${CMAKE_CURRENT_SOURCE_DIR}/unixTime.c
${CMAKE_CURRENT_SOURCE_DIR}/../../lib/crc/nf_crc16.c
${CMAKE_CURRENT_SOURCE_DIR}/../../events/ble/ble_data_event.c
${CMAKE_CURRENT_SOURCE_DIR}/../../events/ble/ble_cmd_event.c
${CMAKE_CURRENT_SOURCE_DIR}/../../events/ble/ble_ctrl_event.c
${CMAKE_CURRENT_SOURCE_DIR}/../../events/lte/lte_proto_event.c
${CMAKE_CURRENT_SOURCE_DIR}/../../events/messaging/messaging_module_events.c
${CMAKE_CURRENT_SOURCE_DIR}/../../events/cellular_controller/cellular_controller_events.c
${CMAKE_CURRENT_SOURCE_DIR}/../../events/gps_controller/gps_controller_events.c
${CMAKE_CURRENT_SOURCE_DIR}/../../events/error_handler/error_event.c
${CMAKE_CURRENT_SOURCE_DIR}/../../events/storage/storage_event.c
)

add_dependencies(${ZEPHYR_CURRENT_LIBRARY}
        		collar_protocol_headers
    )