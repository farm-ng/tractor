
cmake_minimum_required(VERSION 3.17)

project(MyProject)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(farm_ng_core REQUIRED)
find_package(farm_ng_examples REQUIRED)
find_package(farm_ng_perception REQUIRED)
find_package(farm_ng_calibration REQUIRED)
find_package(farm_ng_tractor REQUIRED)

add_executable(helloworld helloworld.cpp)
target_link_libraries(helloworld farm_ng::farm_ng_examples_protobuf farm_ng::farm_ng_core)
