zephyr_include_directories(
        .
        ./helpers
        ./../../events/cellular_controller/
        ./../../events/messaging/
        ./../../events/fw_upgrade/
        ./../../events/error_handler/
        ./../../events/power_manager/
        ../../include
        )

target_sources(app PRIVATE
        cellular_controller.c
        ./../../events/cellular_controller/cellular_controller_events.c
        ./helpers/cellular_helpers.c
        )
