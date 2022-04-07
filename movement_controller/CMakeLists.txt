

zephyr_include_directories(
    .
    ./../../events/movement_controller/
    ./../../lib/fifo
    ./../../lib/trigonometry)

target_sources(app PRIVATE 
./../../lib/fifo/nf_fifo.c
./../../lib/trigonometry/trigonometry.c
movement_controller.c
./../../events/movement_controller/movement_events.c
)