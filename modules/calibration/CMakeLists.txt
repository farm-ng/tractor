cmake_minimum_required(VERSION 3.10)
farm_ng_project(calibration)

add_subdirectory(protos)
add_subdirectory(cpp/farm_ng)

farm_ng_export_module()
