add_library(farm-ng-ipc SHARED
  blobstore.cpp
  blobstore.h
  event_log_reader.cpp
  event_log_reader.h
  event_log.cpp
  event_log.h
  image_loader.cpp
  image_loader.h
  init.cpp
  init.h
  ipc.cpp
  ipc.h
  video_streamer.h
  video_streamer.cpp
)
target_link_libraries(farm-ng-ipc farm-ng-protobuf gflags dl ${OpenCV_LIBS} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${GLOG_LIBRARIES})

add_subdirectory(calibration)

add_executable(ipc_logger ipc_logger.cpp)
target_link_libraries(ipc_logger farm-ng-ipc farm-ng-protobuf ${Boost_SYSTEM_LIBRARY})

add_executable(log_playback log_playback.cpp)
target_link_libraries(log_playback farm-ng-ipc farm-ng-protobuf ${Boost_SYSTEM_LIBRARY} gflags)

add_executable(tracking_camera tracking_camera.cpp)
target_link_libraries(tracking_camera farm-ng-ipc farm-ng-protobuf farm-ng-calibration  ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${realsense2_LIBRARY}  ${OpenCV_LIBS} Sophus::Sophus ${GLOG_LIBRARIES})

add_executable(capture_calibration_dataset capture_calibration_dataset.cpp)
target_link_libraries(capture_calibration_dataset farm-ng-ipc farm-ng-protobuf gflags)

add_executable(capture_video_dataset capture_video_dataset.cpp)
target_link_libraries(capture_video_dataset farm-ng-ipc farm-ng-protobuf gflags)

add_executable(calibrate_apriltag_rig calibrate_apriltag_rig.cpp)
target_link_libraries(calibrate_apriltag_rig farm-ng-ipc farm-ng-calibration farm-ng-protobuf gflags)

add_executable(calibrate_base_to_camera calibrate_base_to_camera.cpp)
target_link_libraries(calibrate_base_to_camera farm-ng-ipc farm-ng-calibration farm-ng-protobuf gflags)

add_executable(tracking_study tracking_study.cpp)
target_link_libraries(tracking_study farm-ng-ipc farm-ng-calibration farm-ng-protobuf gflags ${OpenCV_LIBS})
