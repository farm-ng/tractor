find_package(apriltag REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Sophus REQUIRED)

foreach(src_prefix
   apriltag
   camera_model
   camera_pipeline
   pose_utils
   time_series
   camera_pipeline_utils
)
  list(APPEND cpp_files ${src_prefix}.cpp)
  list(APPEND h_files ${src_prefix}.h)
  list(APPEND test_files ${src_prefix}_test.cpp)
endforeach()

add_library(farm_ng_perception SHARED
  frame_grabber.h
  frame_grabber.cpp
  frame_grabber_intel.cpp
  image_loader.cpp
  image_loader.h
  video_streamer.h
  video_streamer.cpp
  eigen_cv.h
  pose_graph.h
  ${h_files}
  ${cpp_files}
  )

target_include_directories(
  farm_ng_perception
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../..
  ${EIGEN3_INCLUDE_DIRS})

target_link_libraries(
  farm_ng_perception
  farm_ng_perception_protobuf
  farm_ng_core
  apriltag::apriltag
  opencv_imgcodecs
  ${OpenCV_LIBS}
  Sophus::Sophus
  ${realsense2_LIBRARY})


# TODO add tests!
# add_executable(farm_ng_perception-test
#   ${test_files}
#  )

# target_link_libraries(farm_ng_perception-test
#     farm_ng_perception
#     gtest gmock gtest_main
#  )
# gtest_discover_tests(farm_ng_perception-test)
