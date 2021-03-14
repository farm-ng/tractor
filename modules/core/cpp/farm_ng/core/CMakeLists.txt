find_package(Boost REQUIRED system filesystem)
find_package(gflags REQUIRED COMPONENTS shared)
find_package(Glog REQUIRED)

set(_CPP)
set(_HEADERS)
foreach(x blobstore event_log_reader event_log init ipc thread_pool)
list(APPEND _CPP ${x}.cpp)
list(APPEND _HEADERS ${x}.h)
endforeach()

farm_ng_add_library(farm_ng_core
  SOURCES ${_CPP}
  HEADERS ${_HEADERS}
  LINK_LIBRARIES
    farm_ng_core_protobuf
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    dl
    pthread
    gflags
    ${GLOG_LIBRARIES}
)

# TODO add tests!
enable_testing()
include(GoogleTest)
