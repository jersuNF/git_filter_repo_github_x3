list(APPEND ZEPHYR_EXTRA_MODULES
        ${CMAKE_CURRENT_SOURCE_DIR}/../collar_protocol/
        )

target_include_directories(app PRIVATE 
	.
	./../../events/error_handler/
	./include/
	./../../events/messaging/
	./battery/
	./charging/
)

target_sources(app PRIVATE
		     ${CMAKE_CURRENT_SOURCE_DIR}/pwr_module.c
		     ${CMAKE_CURRENT_SOURCE_DIR}/battery/battery.c
		     ${CMAKE_CURRENT_SOURCE_DIR}/charging/charging.c
		     ./../../events/error_handler/error_event.c
		     ./../../events/messaging/messaging_module_events.c
)



add_dependencies(${ZEPHYR_CURRENT_LIBRARY}
        		collar_protocol_headers
    )