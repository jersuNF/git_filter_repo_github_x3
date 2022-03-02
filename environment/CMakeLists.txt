zephyr_include_directories(
    .
    ./../../events/env_sensor_event/
    ./../../events/error_handler/)

target_sources(app PRIVATE 
env_sensor.c
./../../events/environment/env_sensor_event.c
./../../events/error_handler/error_event.c
)