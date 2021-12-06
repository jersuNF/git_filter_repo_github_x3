#
# Copyright (c) 2021 Nofence
#

target_include_directories(app PRIVATE .)
target_sources(app PRIVATE
			${CMAKE_CURRENT_SOURCE_DIR}/storage_simulated.c
			${CMAKE_CURRENT_SOURCE_DIR}/modules_common.c
)
