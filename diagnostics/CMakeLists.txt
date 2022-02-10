
if (CONFIG_DIAGNOSTICS)
zephyr_include_directories(
    .
    ./../../events/diagnostics/)

target_sources(app PRIVATE 
diagnostics.c
parser.c
passthrough.c
./../../events/diagnostics/diagnostics_events.c
)
endif()