

zephyr_include_directories(
    .
    ./../../events/movement_controller/)

target_sources(app PRIVATE 
movement_controller.c
./../../events/movement_controller/movement_events.c
)