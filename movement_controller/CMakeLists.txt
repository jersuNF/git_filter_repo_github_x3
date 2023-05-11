

zephyr_include_directories(
    .
    ./../../events/movement_controller/
    ./../../lib/fifo
    ./../../lib/trigonometry
    ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/nclogs/
    )

target_sources(app PRIVATE 
./../../lib/fifo/nf_fifo.c
./../../lib/trigonometry/trigonometry.c
movement_controller.c
./../../events/movement_controller/movement_events.c
)