macro(farm_ng_project name)
  project(farm_ng_${name})
  set(FARM_NG_MODULE_NAME ${name})
  set(FARM_NG_PROJECT_NAME farm_ng_${name})
endmacro()

macro(farm_ng_add_library target)
  set(one_value_args BUILD_INCLUDE_PATH)

  set(multi_value_args
    SOURCES
    HEADERS
    LINK_LIBRARIES
    PUBLIC_INCLUDE_DIRS
    PRIVATE_LINK_LIBRARIES)

  cmake_parse_arguments(FARM_NG_ADD_LIBRARY "" "${one_value_args}" "${multi_value_args}" ${ARGN})

  if(NOT DEFINED FARM_NG_ADD_LIBRARY_BUILD_INCLUDE_PATH)
    set(FARM_NG_ADD_LIBRARY_BUILD_INCLUDE_PATH ${CMAKE_CURRENT_LIST_DIR}/../../ )
  endif()


  add_library(${target} SHARED
    ${FARM_NG_ADD_LIBRARY_SOURCES}
    ${FARM_NG_ADD_LIBRARY_HEADERS}
    )
  add_library(farm_ng::${target} ALIAS ${target})

  target_include_directories(${target} PUBLIC INTERFACE
    $<BUILD_INTERFACE:${FARM_NG_ADD_LIBRARY_BUILD_INCLUDE_PATH}>
    $<INSTALL_INTERFACE:include>
    )

  target_include_directories(${target} PRIVATE
    ${FARM_NG_ADD_LIBRARY_BUILD_INCLUDE_PATH}
    )

  if(DEFINED FARM_NG_ADD_LIBRARY_PUBLIC_INCLUDE_DIRS)
    target_include_directories(${target} PUBLIC
      ${FARM_NG_ADD_LIBRARY_PUBLIC_INCLUDE_DIRS}
      )
  endif()

  set_property(TARGET ${target} PROPERTY VERSION ${farm_ng_VERSION})
  set_property(TARGET ${target} PROPERTY SOVERSION ${farm_ng_MAJOR_VERSION})
  set_property(TARGET ${target} PROPERTY
    INTERFACE_${target}_MAJOR_VERSION ${farm_ng_MAJOR_VERSION})
  set_property(TARGET ${target} APPEND PROPERTY
    COMPATIBLE_INTERFACE_STRING farm_ng_MAJOR_VERSION
    )

  target_link_libraries(
    ${target} PUBLIC
    ${FARM_NG_ADD_LIBRARY_LINK_LIBRARIES}
    )

  target_link_libraries(
    ${target} PRIVATE
    ${FARM_NG_ADD_LIBRARY_PRIVATE_LINK_LIBRARIES}
    )

  install(TARGETS ${target} EXPORT ${FARM_NG_PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
    )

  install(
    FILES ${FARM_NG_ADD_LIBRARY_HEADERS}
    DESTINATION include/farm_ng/${FARM_NG_MODULE_NAME}
    COMPONENT Devel
    )
endmacro()

macro(farm_ng_add_executable target)
  add_executable(${target} ${target}.cpp)
      
  install(TARGETS ${target} EXPORT ${FARM_NG_PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
    )
endmacro()

macro(farm_ng_add_protobufs target)
  set(multi_value_args PROTO_FILES DEPENDENCIES)
  cmake_parse_arguments(FARM_NG_ADD_PROTOBUFS "" "" "${multi_value_args}" ${ARGN})
  set(${target}_PROTOBUF_IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR} CACHE STRING "Path to this project's protobuf sources")
  foreach(_dep_target ${FARM_NG_ADD_PROTOBUFS_DEPENDENCIES})
    list(APPEND DEP_PROTO_INCLUDES  -I ${${_dep_target}_PROTOBUF_IMPORT_DIRS})
  endforeach()

  # Create build directories for all supported languages
  list(APPEND _proto_languages cpp python go ts)
  foreach(_proto_language ${_proto_languages})
    set("_proto_output_dir_${_proto_language}" ${CMAKE_CURRENT_BINARY_DIR}/${_proto_language})
    file(MAKE_DIRECTORY "${_proto_output_dir_${_proto_language}}")
  endforeach()

  # Extract the module name from the target
  string(REGEX REPLACE "farm_ng_|_protobuf" "" _module ${target})
  set(_cpp_out_sources)
  set(_cpp_out_headers)
  set(_cpp_grpc_out_sources)
  set(_cpp_grpc_out_headers)
  foreach (_proto_path ${FARM_NG_ADD_PROTOBUFS_PROTO_FILES})
    SET(_full_proto_path ${CMAKE_CURRENT_SOURCE_DIR}/${_proto_path})
    get_filename_component(_file_we ${_proto_path} NAME_WE)
    get_filename_component(_file_dir ${_proto_path} DIRECTORY)

    # cpp
    set("_protoc_args_cpp"
      "--cpp_out=${_proto_output_dir_cpp}"
      "--grpc_out=${_proto_output_dir_cpp}"
      "--plugin=protoc-gen-grpc=${_GRPC_CPP_PLUGIN_EXECUTABLE}")
    SET(_cpp_out_src ${_proto_output_dir_cpp}/${_file_dir}/${_file_we}.pb.cc)
    SET(_cpp_out_hdr ${_proto_output_dir_cpp}/${_file_dir}/${_file_we}.pb.h)
    SET(_cpp_grpc_out_src ${_proto_output_dir_cpp}/${_file_dir}/${_file_we}.grpc.pb.cc)
    SET(_cpp_grpc_out_hdr ${_proto_output_dir_cpp}/${_file_dir}/${_file_we}.grpc.pb.h)
    list(APPEND _cpp_out_all ${_cpp_out_src} ${_cpp_out_hdr})
    list(APPEND _cpp_out_sources ${_cpp_out_src})
    list(APPEND _cpp_out_headers ${_cpp_out_hdr})
    list(APPEND _cpp_grpc_out_headers ${_cpp_grpc_out_hdr})
    list(APPEND _cpp_grpc_out_sources ${_cpp_grpc_out_src})
    list(APPEND _cpp_grpc_out_all ${_cpp_grpc_out_src} ${_cpp_grpc_out_hdr})
    add_custom_command(
      OUTPUT ${_cpp_out_src} ${_cpp_out_hdr} ${_cpp_grpc_out_src} ${_cpp_grpc_out_hdr}
      COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS ${_protoc_args_cpp} -I ${CMAKE_CURRENT_SOURCE_DIR} ${DEP_PROTO_INCLUDES} ${_full_proto_path}
      DEPENDS ${_full_proto_path} ${PROTOBUF_PROTOC_EXECUTABLE}
      COMMENT "Generating cpp protobuf code for ${_proto_path}"
      VERBATIM)

    # python
    set("_protoc_args_python"
      "--python_out=${_proto_output_dir_python}"
      "--grpc_python_out=${_proto_output_dir_python}"
      "--plugin=protoc-gen-grpc_python=${_GRPC_PYTHON_PLUGIN_EXECUTABLE}")
    SET(_python_out ${_proto_output_dir_python}/${_file_dir}/${_file_we}_pb2.py)
    SET(_python_grpc_out ${_proto_output_dir_python}/${_file_dir}/${_file_we}_pb2_grpc.py)
    list(APPEND _python_out_all ${_python_out})
    list(APPEND _python_grpc_out_all  ${_python_grpc_out})
    add_custom_command(
      OUTPUT ${_python_out} ${_python_grpc_out}
      COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS ${_protoc_args_python} -I ${CMAKE_CURRENT_SOURCE_DIR} ${DEP_PROTO_INCLUDES} ${_full_proto_path}
      DEPENDS ${_full_proto_path} ${PROTOBUF_PROTOC_EXECUTABLE}
      COMMENT "Generating python protobuf code for ${_proto_path}"
      VERBATIM)

    # go

    # With this configuration, .pb.go and .twirp.go files are generated in
    # ${_proto_output_dir_go}/github.com/farm-ng/genproto/${_module}.
    # The fully qualified directory structure isn't necessary for Go modules,
    # but the Twirp generator doesn't yet support --go_opt=module
    # (https://github.com/twitchtv/twirp/issues/226).
    set("_protoc_args_go" "--plugin=/farm_ng/env/go/bin/protoc-gen-go" "--plugin=/farm_ng/env/go/bin/protoc-gen-twirp" "--go_out=${_proto_output_dir_go}" "--twirp_out=${_proto_output_dir_go}")
    SET(_go_out_go "${_proto_output_dir_go}/github.com/farm-ng/genproto/${_module}/${_file_we}.pb.go")
    SET(_go_out_twirp "${_proto_output_dir_go}/github.com/farm-ng/genproto/${_module}/${_file_we}.twirp.go")
    list(APPEND _go_out_all ${_go_out_go} ${_go_out_twirp})
    add_custom_command(
      OUTPUT ${_go_out_go} ${_go_out_twirp}
      COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS ${_protoc_args_go} -I ${CMAKE_CURRENT_SOURCE_DIR} ${DEP_PROTO_INCLUDES} ${_full_proto_path}
      DEPENDS ${_full_proto_path} ${PROTOBUF_PROTOC_EXECUTABLE}
      COMMENT "Generating go protobuf code for ${_proto_path}"
      VERBATIM)

    # ts
    # - forceLong=long provides support for (u)int64s, parsing them as Longs
    # - outputClientImpl=false disables gRPC/twirp client stub generation, which
    #   appears to be broken (generates non-compiling code for bidi streaming services),
    #   and we don't use anyways.
    set("_protoc_args_ts" "--ts_proto_out=forceLong=long,outputClientImpl=false:${_proto_output_dir_ts}")
    SET(_ts_out ${_proto_output_dir_ts}/${_file_dir}/${_file_we}.ts)
    list(APPEND _ts_out_all ${_ts_out})
    add_custom_command(
      OUTPUT ${_ts_out}
      COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS ${_protoc_args_ts} -I ${CMAKE_CURRENT_SOURCE_DIR} ${DEP_PROTO_INCLUDES} ${_full_proto_path}
      DEPENDS ${_full_proto_path} ${PROTOBUF_PROTOC_EXECUTABLE}
      COMMENT "Generating ts protobuf code for ${_proto_path}"
      VERBATIM)
  endforeach()

  # Build grpc libraries in separate targets
  string(REGEX REPLACE "protobuf" "grpc" _grpc_target ${target})

  if(NOT DISABLE_PROTOC_cpp)
    farm_ng_add_library(${target} SOURCES ${_cpp_out_sources}
    HEADERS ${_cpp_out_headers}
    PUBLIC_INCLUDE_DIRS ${Protobuf_INCLUDE_DIRS}
    BUILD_INCLUDE_PATH ${_proto_output_dir_cpp}
    LINK_LIBRARIES  ${Protobuf_LIBRARIES} ${FARM_NG_ADD_PROTOBUFS_DEPENDENCIES})

    farm_ng_add_library(${_grpc_target}
     SOURCES ${_cpp_grpc_out_sources}
     HEADERS ${_cpp_grpc_out_headers}
     BUILD_INCLUDE_PATH ${_proto_output_dir_cpp}
     LINK_LIBRARIES ${target} gRPC::grpc++)

endif()

  if(NOT DISABLE_PROTOC_python)
    add_custom_target(${target}_py ALL DEPENDS ${_python_out_all})
    add_custom_target(${_grpc_target}_py ALL DEPENDS ${_python_grpc_out_all})
  endif()

  if(NOT DISABLE_PROTOC_go)
    add_custom_target(${target}_go ALL DEPENDS ${_go_out_all})
    # Copy a go.mod file to expose the generated Go code as a Go module
    configure_file(
      ${CMAKE_HOME_DIRECTORY}/cmake/go.mod.template
      ${_proto_output_dir_go}/github.com/farm-ng/genproto/${_module}/go.mod)
  endif()

  # ts
  if(NOT DISABLE_PROTOC_ts)
    add_custom_target(${target}_ts ALL DEPENDS ${_ts_out_all})
    # Copy a package.json file to expose the generated TS code as an NPM package
    configure_file(
      ${CMAKE_HOME_DIRECTORY}/cmake/package.json.template
      ${_proto_output_dir_ts}/package.json)
  endif()

endmacro()


macro(farm_ng_export_module)
set(module_name ${FARM_NG_PROJECT_NAME})

set(_EXPORT_DIR ${CMAKE_BINARY_DIR}/export/${module_name})

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${_EXPORT_DIR}/${module_name}ConfigVersion.cmake"
  VERSION ${farm_ng_VERSION}
  COMPATIBILITY AnyNewerVersion
)

export(EXPORT ${module_name}Targets
  FILE "${_EXPORT_DIR}/${module_name}Targets.cmake"
  NAMESPACE farm_ng::
)

configure_file(cmake/${module_name}Config.cmake
  "${_EXPORT_DIR}/${module_name}Config.cmake"
  COPYONLY
)

set(ConfigPackageLocation lib/cmake/${module_name})
install(EXPORT ${module_name}Targets
  FILE
  ${module_name}Targets.cmake
  NAMESPACE
    farm_ng::
  DESTINATION
    ${ConfigPackageLocation}
)
install(
  FILES
    cmake/${module_name}Config.cmake
    "${_EXPORT_DIR}/${module_name}ConfigVersion.cmake"
  DESTINATION
    ${ConfigPackageLocation}
  COMPONENT
    Devel
)

endmacro()
