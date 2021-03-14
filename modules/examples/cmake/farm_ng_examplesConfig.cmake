include(CMakeFindDependencyMacro)

if (NOT TARGET farm_ng::farm_ng_examples_protobuf)
  include("${CMAKE_CURRENT_LIST_DIR}/farm_ng_examplesTargets.cmake")
endif()
