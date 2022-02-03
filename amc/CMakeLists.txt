zephyr_include_directories(
    .
    ./../../events/amc/
    ./../../events/buzzer/)

target_sources(app PRIVATE 
./amc_handler.c
./../../events/amc/request_events.c
./../../events/buzzer/sound_event.c
)