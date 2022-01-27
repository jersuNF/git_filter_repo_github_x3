list(APPEND ZEPHYR_EXTRA_MODULES
        ${CMAKE_CURRENT_SOURCE_DIR}/../collar_protocol/
        )

zephyr_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../fw_upgrade/http_download/
    ${CMAKE_CURRENT_SOURCE_DIR}/../../events/ble/
    ${CMAKE_CURRENT_SOURCE_DIR}/../../events/lte/)

target_sources(app PRIVATE 
${CMAKE_CURRENT_SOURCE_DIR}/messaging.c
${CMAKE_CURRENT_SOURCE_DIR}/../fw_upgrade/http_download/http_downloader.c
${CMAKE_CURRENT_SOURCE_DIR}/../../events/ble/ble_data_event.c
${CMAKE_CURRENT_SOURCE_DIR}/../../events/ble/ble_ctrl_event.c
${CMAKE_CURRENT_SOURCE_DIR}/../../events/lte/lte_proto_event.c
)

add_dependencies(${ZEPHYR_CURRENT_LIBRARY}
        		collar_protocol_headers
    )