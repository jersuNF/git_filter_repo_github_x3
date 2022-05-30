
if (CONFIG_DIAGNOSTICS)
zephyr_include_directories(
    .
    ./commander
    ./../../events/diagnostics/
    ./../../modules/buzzer)

target_sources(app PRIVATE 
    diagnostics.c
    parser.c
    passthrough.c
    ./../../events/diagnostics/diagnostics_events.c
    log_backend_diag.c
)

if (CONFIG_DIAGNOSTICS_TEXT_PARSER)
target_sources(app PRIVATE parser.c)
else()
target_sources(app PRIVATE 
    ./commander/commander.c
    ./commander/cobs.c
    ./commander/cmd_settings.c
    ./commander/cmd_stimulator.c
    ./commander/cmd_storage.c
    ./commander/cmd_system.c
)
endif()

endif()
