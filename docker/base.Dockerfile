FROM farmng/devel@sha256:d152a1c90f8d52e218653b8f8f82ea13b80621a27a17c733794ad070fb3dd809

WORKDIR /farm_ng
RUN export FARM_NG_ROOT=/farm_ng

# Build first-party c++
COPY CMakeLists.txt .
COPY cmake cmake
COPY doc doc
COPY modules modules
SHELL ["/bin/bash", "-c"]
RUN	. setup.bash && \
  mkdir -p build && \
  cd build && \
  cmake -DCMAKE_PREFIX_PATH=`pwd`/../env -DBUILD_DOCS=TRUE -DCMAKE_BUILD_TYPE=Release -DDISABLE_PROTOC_ts=TRUE -DDISABLE_PROTOC_go=TRUE .. && \
  make docs && \
  make -j`nproc --ignore=1`

# TODO(isherman): Reduce size of final image with multi-stage build
# https://devblogs.microsoft.com/cppblog/using-multi-stage-containers-for-c-development/
