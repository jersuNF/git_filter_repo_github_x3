zephyr_include_directories(
    .
    ./../../events/amc/
    ./../../events/storage_controller/
    ./../../events/storage_controller/include
    ./../../events/buzzer/)

target_sources(app PRIVATE 
./amc_handler.c
./../../events/amc/request_events.c
./../../events/storage_controller/storage_events.c
./../../events/storage_controller/pasture_event.c
./../../events/buzzer/sound_event.c
)