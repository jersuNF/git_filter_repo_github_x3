

target_include_directories(app PRIVATE 
	.
	./../../events/error_handler/
	./../../events/messaging/

)

target_sources(app PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/ep_module.c
		./../../events/error_handler/error_event.c
		./../../events/messaging/messaging_module_events.c
)
