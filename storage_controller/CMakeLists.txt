#
# Copyright (c) 2021 Nofence
#

target_include_directories(app PRIVATE 
			.
			./include/
			./../../events/storage_controller/)

target_sources(app PRIVATE
			./storage.c
			./../../events/storage_controller/storage_events.c
)
