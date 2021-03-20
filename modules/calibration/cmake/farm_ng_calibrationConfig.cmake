include(CMakeFindDependencyMacro)
find_dependency(Ceres)
find_dependency(farm_ng_core)
find_dependency(farm_ng_perception)

if (NOT TARGET farm_ng::farm_ng_calibration)
  include("${CMAKE_CURRENT_LIST_DIR}/farm_ng_calibrationTargets.cmake")
endif()
