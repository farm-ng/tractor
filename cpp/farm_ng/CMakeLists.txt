add_library(farm-ng-ipc SHARED
  ipc.h
  ipc.cpp
  event_log.h
  event_log.cpp
  event_log_reader.h
  event_log_reader.cpp
  )
target_link_libraries(farm-ng-ipc farm-ng-protobuf ${Boost_SYSTEM_LIBRARY})

add_executable(farm-ng-ipc-logger ipc_logger.cpp)
target_link_libraries(farm-ng-ipc-logger farm-ng-ipc farm-ng-protobuf ${Boost_SYSTEM_LIBRARY})

add_executable(farm-ng-log-playback log_playback.cpp)
target_link_libraries(farm-ng-log-playback farm-ng-ipc farm-ng-protobuf ${Boost_SYSTEM_LIBRARY})

add_executable(farm-ng-tracking_camera tracking_camera.cpp)
target_link_libraries(farm-ng-tracking_camera farm-ng-ipc farm-ng-protobuf ${Boost_SYSTEM_LIBRARY} ${realsense2_LIBRARY}  ${OpenCV_LIBS} )
