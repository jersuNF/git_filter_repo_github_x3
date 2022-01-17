zephyr_include_directories(
        .
        ./../../events/cellular_controller/)

target_sources(app PRIVATE
        cellular_controller.c
        ./../../events/cellular_controller/cellular_controller_events.c
        cellular_helpers.c
        )