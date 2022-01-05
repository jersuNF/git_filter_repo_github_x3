

zephyr_include_directories(
    .
    ./http_download)

target_sources(app PRIVATE 
fw_upgrade.c
fw_upgrade_events.c
./http_download/http_downloader.c
)