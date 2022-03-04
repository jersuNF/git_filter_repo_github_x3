

target_include_directories(app PRIVATE 
	.
	./../../events/error_handler/
	./battery/
	./charging/
)

target_sources(app PRIVATE
		     ${CMAKE_CURRENT_SOURCE_DIR}/pwr_module.c
		     ${CMAKE_CURRENT_SOURCE_DIR}/battery/battery.c
		     ${CMAKE_CURRENT_SOURCE_DIR}/charging/charging.c
		     ./../../events/error_handler/error_event.c
)
