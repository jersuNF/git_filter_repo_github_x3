#
# Copyright (c) 2021 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

target_include_directories(app PRIVATE .
                        ./../../events/error_handler/
)
target_sources(app PRIVATE nofence_watchdog.c)
