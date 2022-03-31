

zephyr_include_directories(
    .
    ./../../events/error_handler/
    ./../../events/watchdog/
    ./../../events/pwr_event/
    )

target_sources(app PRIVATE 
error_handler.c
./../../events/error_handler/error_event.c
)