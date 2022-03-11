list(APPEND ZEPHYR_EXTRA_MODULES
        ${CMAKE_CURRENT_SOURCE_DIR}/../collar_protocol/
        )

zephyr_include_directories(
    ./include/
    ./../../../modules/storage_controller/include/)

target_sources(app PRIVATE 
./amc_cache.c
./amc_dist.c
./amc_zone.c
)

add_dependencies(${ZEPHYR_CURRENT_LIBRARY}
        		collar_protocol_headers
    )