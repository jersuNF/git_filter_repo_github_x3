

zephyr_include_directories(
    .
    ./../../events/error_handler/)

target_sources(app PRIVATE 
error-handler.c
./../../events/error_handler/error_handler_events.c
)