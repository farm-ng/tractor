add_library(farm_ng_tractor SHARED
  base_to_camera_calibrator.h
  base_to_camera_calibrator.cpp
)
target_include_directories(farm_ng_tractor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../..)
target_link_libraries(farm_ng_tractor farm_ng_tractor_protobuf farm_ng_calibration)

# TODO add tests!
enable_testing()
include(GoogleTest)
