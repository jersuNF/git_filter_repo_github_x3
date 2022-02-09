zephyr_include_directories(
    .
    ./../../events/env_sensor_event/)

target_sources(app PRIVATE 
env_sensor.c
./../../events/environment/env_sensor_event.c
)