
foreach(src_prefix
   apriltag
   apriltag_rig_calibrator
   base_to_camera_calibrator
   camera_model
   local_parameterization_se3
   pose_utils
   solver
)
list(APPEND cpp_files ${src_prefix}.cpp)
list(APPEND h_files ${src_prefix}.h)
list(APPEND test_files ${src_prefix}_test.cpp)
endforeach()

add_library(farm-ng-calibration
  ${cpp_files}
  ${h_files}
)

target_link_libraries(farm-ng-calibration
   ${CERES_LIBRARIES} ${GLOG_LIBRARIES} Sophus::Sophus opencv_imgcodecs farm-ng-protobuf farm-ng-ipc)


add_executable(farm_ng_calibration_test
  ${test_files}
)

target_link_libraries(farm_ng_calibration_test
    farm-ng-calibration
    gtest gmock gtest_main
)
gtest_discover_tests(farm_ng_calibration_test)


add_executable(base_to_camera_calibrator_driver
base_to_camera_calibrator_driver.cpp)

target_link_libraries(base_to_camera_calibrator_driver farm-ng-calibration gflags )
