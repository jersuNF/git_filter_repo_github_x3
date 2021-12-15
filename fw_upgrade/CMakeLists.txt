zephyr_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/http_download)

target_sources(app PRIVATE 
${CMAKE_CURRENT_SOURCE_DIR}/fw_upgrade.c
${CMAKE_CURRENT_SOURCE_DIR}/fw_upgrade_events.c
${CMAKE_CURRENT_SOURCE_DIR}/http_download/http_downloader.c
)