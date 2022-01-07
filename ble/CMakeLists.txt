#
# Copyright (c) 2021 Nofence
#

target_include_directories(app PRIVATE .)
target_sources(app PRIVATE
			${CMAKE_CURRENT_SOURCE_DIR}/ble_handler.c
			${CMAKE_CURRENT_SOURCE_DIR}/ble_dfu.c

)
