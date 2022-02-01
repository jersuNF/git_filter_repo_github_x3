zephyr_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../events/buzzer)

target_sources(app PRIVATE 
${CMAKE_CURRENT_SOURCE_DIR}/buzzer.c
${CMAKE_CURRENT_SOURCE_DIR}/../../events/buzzer/sound_event.c
)