zephyr_include_directories(
    .)

target_sources(app PRIVATE 
./amc_data.c
./amc_dist.c
./amc_zone.c
)