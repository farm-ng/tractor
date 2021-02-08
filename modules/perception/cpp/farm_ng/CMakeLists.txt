add_subdirectory(perception)

add_executable(capture_video_dataset capture_video_dataset.cpp)
target_link_libraries(capture_video_dataset farm_ng_perception)

add_executable(create_video_dataset create_video_dataset.cpp)
target_link_libraries(create_video_dataset farm_ng_perception)

add_executable(detect_apriltags detect_apriltags.cpp)
target_link_libraries(detect_apriltags farm_ng_perception)

add_executable(camera_pipeline camera_pipeline.cpp)
target_link_libraries(camera_pipeline farm_ng_perception)

find_package(realsense2)
if(${realsense2_FOUND})
  add_executable(convert_rs2_bag convert_rs2_bag.cpp)
  target_link_libraries(convert_rs2_bag farm_ng_perception ${realsense2_LIBRARY})
endif()
