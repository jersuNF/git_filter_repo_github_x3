#
# Copyright (c) 2021 Nofence
#

zephyr_include_directories(.
			./services
			${CMAKE_CURRENT_SOURCE_DIR}/../../../include
			${CMAKE_CURRENT_SOURCE_DIR}/../../events/ble
			${CMAKE_CURRENT_SOURCE_DIR}/../../events/messaging
			${CMAKE_CURRENT_SOURCE_DIR}/../../events/watchdog
  			${CMAKE_CURRENT_SOURCE_DIR}/../../modules/nf_settings/include
			../../../include)
			
target_sources(app PRIVATE
			${CMAKE_CURRENT_SOURCE_DIR}/ble_controller.c
			${CMAKE_CURRENT_SOURCE_DIR}/ble_dfu.c
			${CMAKE_CURRENT_SOURCE_DIR}/beacon_processor.c
			${CMAKE_CURRENT_SOURCE_DIR}/services/nofence_service.c
			${CMAKE_CURRENT_SOURCE_DIR}/../../events/ble/ble_conn_event.c
			${CMAKE_CURRENT_SOURCE_DIR}/../../events/ble/ble_ctrl_event.c
			${CMAKE_CURRENT_SOURCE_DIR}/../../events/ble/ble_data_event.c
			${CMAKE_CURRENT_SOURCE_DIR}/../../events/messaging/msg_data_event.c
			${CMAKE_CURRENT_SOURCE_DIR}/../../events/watchdog/watchdog_event.c
)
