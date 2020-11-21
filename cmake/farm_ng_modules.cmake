find_package(Protobuf REQUIRED)

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

  foreach (_proto_path ${FARM_NG_ADD_PROTOBUFS_PROTO_FILES})
    SET(_full_proto_path ${CMAKE_CURRENT_SOURCE_DIR}/${_proto_path})
    get_filename_component(_file_we ${_proto_path} NAME_WE)
    get_filename_component(_file_dir ${_proto_path} DIRECTORY)

    # cpp
    set("_protoc_args_cpp" "--cpp_out=${_proto_output_dir_cpp}")
    SET(_cpp_out_src ${_proto_output_dir_cpp}/${_file_dir}/${_file_we}.pb.cc)
    SET(_cpp_out_hdr ${_proto_output_dir_cpp}/${_file_dir}/${_file_we}.pb.h)
    list(APPEND _cpp_out_all ${_cpp_out_src} ${_cpp_out_hdr})
    add_custom_command(
      OUTPUT ${_cpp_out_src} ${_cpp_out_hdr}
      COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS ${_protoc_args_cpp} -I ${CMAKE_CURRENT_SOURCE_DIR} ${DEP_PROTO_INCLUDES} ${_full_proto_path}
      DEPENDS ${_full_proto_path} ${PROTOBUF_PROTOC_EXECUTABLE}
      COMMENT "Generating cpp protobuf code for ${_proto_path}"
      VERBATIM)

    # python
    set("_protoc_args_python" "--python_out=${_proto_output_dir_python}")
    SET(_python_out ${_proto_output_dir_python}/${_file_dir}/${_file_we}_pb2.py)
    list(APPEND _python_out_all ${_python_out})
    add_custom_command(
      OUTPUT ${_python_out}
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
    set("_protoc_args_go" "--go_out=${_proto_output_dir_go}" "--twirp_out=${_proto_output_dir_go}")
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
    set("_protoc_args_ts" "--ts_proto_out=forceLong=long:${_proto_output_dir_ts}")
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

  if(NOT DISABLE_PROTOC_cpp)
    add_library(${target} SHARED ${_cpp_out_all})
    target_include_directories(${target} PUBLIC ${_proto_output_dir_cpp})
    target_link_libraries(${target} ${Protobuf_LIBRARIES} ${FARM_NG_ADD_PROTOBUFS_DEPENDENCIES})
  endif()

  if(NOT DISABLE_PROTOC_python)
    add_custom_target(${target}_py ALL DEPENDS ${_python_out_all})
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
