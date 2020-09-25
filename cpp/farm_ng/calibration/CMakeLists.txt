
foreach(src_prefix
   apriltag
   solver
)
list(APPEND cpp_files ${src_prefix}.cpp)
list(APPEND h_files ${src_prefix}.h)
list(APPEND test_files ${src_prefix}_test.cpp)
endforeach()

add_library(farm_ng_calibration
  ${cpp_files}
  ${h_files}
)

target_link_libraries(farm_ng_calibration
   ${CERES_LIBRARIES} ${GLOG_LIBRARIES} Sophus::Sophus)


add_executable(farm_ng_calibration_test
  ${test_files}
)

target_link_libraries(farm_ng_calibration_test
    farm_ng_calibration
    gtest gmock gtest_main
)
gtest_discover_tests(farm_ng_calibration_test)
