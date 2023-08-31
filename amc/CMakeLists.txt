list(APPEND ZEPHYR_EXTRA_MODULES
        ${CMAKE_CURRENT_SOURCE_DIR}/../collar_protocol/
        )

zephyr_include_directories(
    .
    ./../../events/amc/
    ./../../modules/storage_controller/
    ./../../events/ble/
    ./../../events/messaging/
    ./../../events/gnss_controller/
    ./../../events/buzzer/
    ./../../lib/trigonometry/
    ./../../lib/fifo/
    ./../../events/error_handler/
    ./../../events/power_manager/
    ./../../events/movement_controller)

target_sources(app PRIVATE 
./amc_handler.c
./../../events/amc/request_events.c
./../../lib/trigonometry/trigonometry.c
./../../lib/fifo/nf_fifo.c
./../../modules/storage_controller/storage.c
./../../events/buzzer/sound_event.c
./../../events/messaging/messaging_module_events.c
./../../events/gnss_controller/gnss_controller_events.c
./../../events/error_handler/error_event.c
./../../events/power_manager/pwr_event.c
)


add_dependencies(${ZEPHYR_CURRENT_LIBRARY}
        		collar_protocol_headers
    )

add_subdirectory(lib)
