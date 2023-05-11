if(CONFIG_NCLOGS)
    list(APPEND ZEPHYR_EXTRA_MODULES 
    ${CMAKE_CURRENT_SOURCE_DIR}/../trice
    ${CMAKE_CURRENT_SOURCE_DIR}/../collar_protocol/
    )
    add_dependencies(${ZEPHYR_CURRENT_LIBRARY}
        		collar_protocol_headers
    )
    target_sources(app PRIVATE
        nclogs.c
    )
endif()
