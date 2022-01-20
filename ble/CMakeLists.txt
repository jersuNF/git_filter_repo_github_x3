#
# Copyright (c) 2021 Nofence
#

zephyr_include_directories(.
			./services)
#target_include_directories(app PRIVATE .)
target_sources(app PRIVATE
			${CMAKE_CURRENT_SOURCE_DIR}/ble_controller.c
			${CMAKE_CURRENT_SOURCE_DIR}/ble_dfu.c
			${CMAKE_CURRENT_SOURCE_DIR}/services/nofence_service.c
)
