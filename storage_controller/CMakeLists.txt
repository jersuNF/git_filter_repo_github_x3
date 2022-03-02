#
# Copyright (c) 2022 Nofence AS
#

target_include_directories(app PRIVATE 
			.
			./include/
			./../../events/error_handler/)

target_sources(app PRIVATE
			./storage.c
			./../../events/error_handler/error_event.c
)
