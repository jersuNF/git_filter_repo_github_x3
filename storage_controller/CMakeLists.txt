#
# Copyright (c) 2022 Nofence AS
#

list(APPEND ZEPHYR_EXTRA_MODULES
        ${CMAKE_CURRENT_SOURCE_DIR}/../collar_protocol/
        )

target_include_directories(app PRIVATE 
			.
			./include/
			./../../events/error_handler/
			./../../events/storage/
			./../../events/messaging/
			./../../modules/messaging/)

target_sources(app PRIVATE
			./storage.c
			./../../events/error_handler/error_event.c
			./../../events/storage/storage_event.c
)


add_dependencies(${ZEPHYR_CURRENT_LIBRARY}
        		collar_protocol_headers
    )