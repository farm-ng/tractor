cmake_minimum_required(VERSION 3.17)

# Configure CCache if available
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

project(farm_ng_root)

execute_process(COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/version.sh OUTPUT_VARIABLE farm_ng_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)
message(STATUS "farm_ng_VERSION: -- ${farm_ng_VERSION} --")
string(REPLACE "." ";" VERSION_LIST ${farm_ng_VERSION})
list(GET VERSION_LIST 0 farm_ng_VERSION_MAJOR)
list(GET VERSION_LIST 1 farm_ng_VERSION_MINOR)
list(GET VERSION_LIST 2 farm_ng_VERSION_PATCH)

include(CMakeToolsHelpers OPTIONAL)

set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH ON)
set(BUILD_SHARED_LIBS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter -Wno-unused-but-set-variable -Wno-unused-variable -Wno-unused-function -Wno-maybe-uninitialized -Wno-implicit-fallthrough -Wno-deprecated-declarations")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(cmake/farm_ng_modules.cmake)
include(cmake/farm_ng_find_base_packages.cmake)

list(APPEND _modules core perception calibration tractor frontend examples)

set(BUILD_ONLY_DOCS FALSE CACHE BOOL "Build only the documentation")

add_subdirectory(doc)
if(BUILD_ONLY_DOCS)
  return()
endif()

if(BUILD_ONLY_PROTO)
  message(STATUS "Build only proto.")
  farm_ng_find_base_packages()
  foreach(_module ${_modules})
    add_subdirectory("modules/${_module}/protos")
  endforeach()
  return()
endif()
option(INSTALL_GTEST "" OFF)
add_subdirectory(third_party/googletest)

farm_ng_find_base_packages()

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
