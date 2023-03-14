target_include_directories(app PRIVATE 
.
./include/)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/nf_settings.c)
