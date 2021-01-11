add_subdirectory(core)

add_executable(ipc_logger ipc_logger.cpp)
target_link_libraries(ipc_logger farm_ng_core)

add_executable(log_playback log_playback.cpp)
target_link_libraries(log_playback farm_ng_core)
