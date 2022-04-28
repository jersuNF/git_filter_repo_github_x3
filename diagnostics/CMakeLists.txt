
if (CONFIG_DIAGNOSTICS)
zephyr_include_directories(
    .
    ./../../events/diagnostics/
    ./../../modules/buzzer)

target_sources(app PRIVATE 
diagnostics.c
parser.c
passthrough.c
./../../events/diagnostics/diagnostics_events.c
)

if (CONFIG_DIAGNOSTICS_TEXT_PARSER)
target_sources(app PRIVATE parser.c)
else()
target_sources(app PRIVATE commander.c cobs.c)
endif()

endif()