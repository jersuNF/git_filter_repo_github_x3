#
# Copyright (c) 2022 Nofence AS
#

target_include_directories(app PRIVATE 
			.
			./include/
			./../../events/error_handler/
			./../../events/storage_controller/)

target_sources(app PRIVATE
			./storage.c
			./../../events/storage_controller/pasture_event.c
			./../../events/error_handler/error_event.c
)
