cmake_minimum_required(VERSION 3.10)
project(farm_ng_root)

include(CMakeToolsHelpers OPTIONAL)

set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH ON)
set(BUILD_SHARED_LIBS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter -Wno-unused-but-set-variable -Wno-unused-variable -Wno-unused-function -Wno-maybe-uninitialized -Wno-implicit-fallthrough")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(cmake/farm_ng_modules.cmake)

list(APPEND _modules core perception calibration tractor frontend)

if(BUILD_ONLY_PROTO)
  message(STATUS "Build only proto.")
  foreach(_module ${_modules})
    add_subdirectory("modules/${_module}/protos")
  endforeach()
  return()
endif()

add_subdirectory(third_party)
add_subdirectory(third_party/googletest)

foreach(_module ${_modules})
  add_subdirectory("modules/${_module}")
endforeach()

# Create targets that can be used to build all protos for a particular language
list(APPEND _proto_languages py go ts)
foreach(_proto_language ${_proto_languages})
  if(NOT "${DISABLE_PROTOC_${_proto_language}}")
    add_custom_target(farm_ng_all_protobuf_${_proto_language})
    foreach(_module ${_modules})
      add_dependencies("farm_ng_all_protobuf_${_proto_language}" "farm_ng_${_module}_protobuf_${_proto_language}")
    endforeach()
  endif()
endforeach()
