find_package(apriltag REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
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

find_package(realsense2 REQUIRED)
if(${realsense2_FOUND})
list(APPEND cpp_files frame_grabber_intel.cpp)
list(APPEND grabber_deps   ${realsense2_LIBRARY})
list(APPEND supported_grabbers realsense2)
endif()
find_package(k4a QUIET)
if(${k4a_FOUND})
message(STATUS "Building with k4a support")
list(APPEND cpp_files frame_grabber_k4a.cpp)
list(APPEND grabber_deps k4a::k4a)
list(APPEND supported_grabbers k4a)
endif()

message(STATUS "Supported frame grabber implementations: ")
foreach(grabber ${supported_grabbers})
  message(STATUS "  ${grabber}")
endforeach(grabber)


farm_ng_add_library(farm_ng_perception
  SOURCES
    frame_grabber.cpp
    frame_grabber_intel.cpp
    image_loader.cpp
    video_streamer.cpp
    point_cloud.cpp
    tensor.cpp
    ${cpp_files}
  HEADERS
    frame_grabber.h
    image_loader.h
    video_streamer.h
    eigen_cv.h
    pose_graph.h
    point_cloud.h
    robot_arm_fk.h
    tensor.h
    ${h_files}
  LINK_LIBRARIES
    farm_ng_perception_protobuf
    farm_ng_core
    apriltag::apriltag
    opencv_imgcodecs
    ${OpenCV_LIBS}
    Sophus::Sophus
  PRIVATE_LINK_LIBRARIES
    ${grabber_deps}
  PUBLIC_INCLUDE_DIRS
    ${EIGEN3_INCLUDE_DIRS}
)
