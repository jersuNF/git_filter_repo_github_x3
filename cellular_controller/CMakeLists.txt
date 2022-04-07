zephyr_include_directories(
        .
        ./helpers
        ./../../events/cellular_controller/
        ./../../events/messaging/
        ./../../events/error_handler/
        ../../include
        ./../../modules/nf_settings/include)

target_sources(app PRIVATE
        cellular_controller.c
        ./../../events/cellular_controller/cellular_controller_events.c
        ./helpers/cellular_helpers.c
        )
