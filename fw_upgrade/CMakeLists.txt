

zephyr_include_directories(
    .
    ./http_download
    ./../../events/fw_upgrade/)

target_sources(app PRIVATE 
fw_upgrade.c
./../../events/fw_upgrade/fw_upgrade_events.c
./http_download/http_downloader.c
)