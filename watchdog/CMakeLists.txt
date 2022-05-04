#
# Copyright (c) 2021 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

target_include_directories(app PRIVATE .
                        ./../../events/watchdog/
                        ./../../events/error_handler/
)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/watchdog_app.c
./../../events/watchdog/watchdog_event.c)
