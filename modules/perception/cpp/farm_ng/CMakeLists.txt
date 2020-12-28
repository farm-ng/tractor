add_subdirectory(perception)

add_executable(capture_video_dataset capture_video_dataset.cpp)
target_link_libraries(capture_video_dataset farm_ng_perception)

add_executable(create_video_dataset create_video_dataset.cpp)
target_link_libraries(create_video_dataset farm_ng_perception)

add_executable(detect_apriltags detect_apriltags.cpp)
target_link_libraries(detect_apriltags farm_ng_perception)

add_executable(camera_pipeline camera_pipeline.cpp)
target_link_libraries(camera_pipeline farm_ng_perception)

# TODO: Should realsense2 symbols be exported from the perception library?
find_package(realsense2 REQUIRED)
if(${realsense2_FOUND})
  add_executable(rs2_bag_to_event_log rs2_bag_to_event_log.cpp)
  target_link_libraries(rs2_bag_to_event_log farm_ng_perception ${realsense2_LIBRARY})
endif()
