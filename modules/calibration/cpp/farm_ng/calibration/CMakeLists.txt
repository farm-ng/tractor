find_package(Ceres REQUIRED)

foreach(src_prefix
      apriltag_rig_calibrator
      epipolar
      flow_book_keeper
      intrinsic_calibrator
      local_parameterization
      multi_view_apriltag_rig_calibrator
      solver
      visual_odometer)
   list(APPEND cpp_files ${src_prefix}.cpp)
   list(APPEND h_files ${src_prefix}.h)
   list(APPEND test_files ${src_prefix}_test.cpp)
endforeach()

farm_ng_add_library(farm_ng_calibration
  SOURCES ${cpp_files}
  HEADERS ${h_files}
  LINK_LIBRARIES
    farm_ng_calibration_protobuf
    farm_ng_perception
    ${CERES_LIBRARIES}
)

farm_ng_add_library(farm_ng_calibration_robot_hal_client
  SOURCES robot_hal_client.cc
  HEADERS robot_hal_client.h
  LINK_LIBRARIES
    farm_ng_calibration_grpc
)
