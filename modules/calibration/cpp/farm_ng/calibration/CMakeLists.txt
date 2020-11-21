find_package(Ceres REQUIRED)

foreach(src_prefix
      apriltag_rig_calibrator
      epipolar
      flow_book_keeper
      local_parameterization
      multi_view_apriltag_rig_calibrator
      solver
      visual_odometer)
   list(APPEND cpp_files ${src_prefix}.cpp)
   list(APPEND h_files ${src_prefix}.h)
   list(APPEND test_files ${src_prefix}_test.cpp)
endforeach()

add_library(farm_ng_calibration
  ${cpp_files}
  ${h_files}
)
target_include_directories(farm_ng_calibration PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../..)
target_link_libraries(farm_ng_calibration farm_ng_calibration_protobuf farm_ng_perception ${CERES_LIBRARIES})


# TODO add tests!
# add_executable(farm_ng_calibration_test
#   ${test_files}
# )

# target_link_libraries(farm_ng_calibration_test
#     farm_ng_calibration
#     gtest gmock gtest_main
# )
# gtest_discover_tests(farm_ng_calibration_test)
