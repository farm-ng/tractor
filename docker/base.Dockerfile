FROM farmng/devel@sha256:0ec40dd3761f54587e25da1eb8a5e6546cf8590e4cb4079d8de19a0e65b724fc

WORKDIR /farm_ng
RUN export FARM_NG_ROOT=/farm_ng

# Build first-party c++
COPY CMakeLists.txt .
COPY cmake cmake
COPY modules modules
SHELL ["/bin/bash", "-c"]
RUN	. setup.bash && \
  mkdir -p build && \
  cd build && \
  cmake -DCMAKE_PREFIX_PATH=`pwd`/../env -DBUILD_DOCS=TRUE -DCMAKE_BUILD_TYPE=Release -DDISABLE_PROTOC_ts=TRUE -DDISABLE_PROTOC_go=TRUE .. && \
  make -j`nproc --ignore=1`

# TODO(isherman): Reduce size of final image with multi-stage build
# https://devblogs.microsoft.com/cppblog/using-multi-stage-containers-for-c-development/
