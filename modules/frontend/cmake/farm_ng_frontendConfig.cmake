include(CMakeFindDependencyMacro)

if (NOT TARGET farm_ng::farm_ng_frontend_protobuf)
  include("${CMAKE_CURRENT_LIST_DIR}/farm_ng_frontendTargets.cmake")
endif()
