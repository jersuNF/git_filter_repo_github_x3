# SPDX-License-Identifier: Apache-2.0

if (CONFIG_COLLAR_PROTOCOL)
    zephyr_include_directories(.)

    # Nanopb definitions
    list (APPEND CMAKE_MODULE_PATH
            "${ZEPHYR_BASE}/../modules/lib/nanopb/extra")

    find_package(Nanopb REQUIRED)
    include_directories(${NANOPB_INCLUDE_DIRS})
    zephyr_include_directories(${NANOPB_INCLUDE_DIRS})
    
    include_directories(${CMAKE_CURRENT_BINARY_DIR})
    zephyr_include_directories(${CMAKE_CURRENT_BINARY_DIR})
    
    # Pick up embedded protocol buffer files from external git project
    set(COLLAR_PROTOCOL_DIR
            ${ZEPHYR_BASE}/../collar-protocol
            )

    nanopb_generate_cpp(PROTO_SRCS PROTO_HDRS
            ${COLLAR_PROTOCOL_DIR}/base.proto
            ${COLLAR_PROTOCOL_DIR}/embedded.proto
            )
    set_source_files_properties(${PROTO_SRCS} ${PROTO_HDRS}
            PROPERTIES GENERATED TRUE)
    
    zephyr_interface_library_named(collar_protocol)
    zephyr_library_include_directories(
            ${NANOPB_INCLUDE_DIRS}
            ${CMAKE_CURRENT_BINARY_DIR}
    )
    zephyr_library_sources(
            collar_protocol.c
            ${PROTO_SRCS}
            ${PROTO_HDRS}
    )

    add_custom_target(
        collar_protocol_headers
        DEPENDS ${PROTO_HDRS}
    )

endif ()
