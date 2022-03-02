zephyr_include_directories(
    .
    ./../../events/amc/
    ./../../modules/storage_controller/
    ./../../events/messaging/
    ./../../events/buzzer/
    ./../../events/error_handler/)

target_sources(app PRIVATE 
./amc_handler.c
./../../events/amc/request_events.c
./../../events/storage_controller/pasture_event.c
./../../modules/storage_controller/storage.c
./../../events/buzzer/sound_event.c
./../../events/messaging/module_module_events.c
./../../events/error_handler/error_event.c
)