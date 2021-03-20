include(CMakeFindDependencyMacro)
find_dependency(Glog)

if (NOT TARGET farm_ng::farm_ng_core)
  include("${CMAKE_CURRENT_LIST_DIR}/farm_ng_coreTargets.cmake")
endif()
