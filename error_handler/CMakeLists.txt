

zephyr_include_directories(
    .
    ./../../events/error_handler/
    ./../../events/messaging/
    ./../../events/watchdog/
    ./../../events/pwr_event/
    ./../../modules/storage_controller/include/
    ./../../modules/storage_controller/

    )

target_sources(app PRIVATE 
error_handler.c
./../../events/error_handler/error_event.c
)