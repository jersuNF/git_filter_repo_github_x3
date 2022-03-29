

target_include_directories(app PRIVATE 
	.
	./../../events/error_handler/
	./../../events/watchdog/
	./battery/
)

target_sources(app PRIVATE
		     ${CMAKE_CURRENT_SOURCE_DIR}/pwr_module.c
		     ${CMAKE_CURRENT_SOURCE_DIR}/battery/battery.c
		     ./../../events/error_handler/error_event.c
		     ./../../events/watchdog/watchdog_event.c
)

#zephyr_library_include_directories(./../../events/watchdog/)