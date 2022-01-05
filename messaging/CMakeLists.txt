


list(APPEND ZEPHYR_EXTRA_MODULES
        ${CMAKE_CURRENT_SOURCE_DIR}/../collar_protocol/
        )

zephyr_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR})

target_sources(app PRIVATE 
${CMAKE_CURRENT_SOURCE_DIR}/messaging.c
)

add_dependencies(${ZEPHYR_CURRENT_LIBRARY}
        		collar_protocol_headers
    )