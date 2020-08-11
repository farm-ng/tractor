cmake_minimum_required(VERSION 3.10)
project(native-tool)

include(CMakeToolsHelpers OPTIONAL)


set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH ON)
set(BUILD_SHARED_LIBS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter -Wno-unused-but-set-variable -Wno-unused-variable -Wno-unused-function -Wno-maybe-uninitialized -Wno-implicit-fallthrough")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Protobuf REQUIRED)
find_package(Boost REQUIRED system filesystem regex)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
include_directories(${GLFW_INCLUDE_DIRS})

include_directories(third_party/imgui)
include_directories(third_party/imgui/examples)
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR}/protos)
add_subdirectory(third_party)
add_subdirectory(protos)
include_directories(cpp)
add_subdirectory(cpp/farm_ng)
