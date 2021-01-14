ARG base_image=ubuntu:18.04
FROM $base_image
RUN apt-get update --fix-missing && \
    apt-get install -y --no-install-recommends \
    autoconf \
    build-essential \
    ca-certificates \
    git \
    libtool \
    pkg-config \
    python3-pip \
    && \
    apt-get clean

RUN pip3 install --upgrade pip && pip3 install cmake
RUN git clone --depth=1 --recurse-submodules -b v1.34.0 https://github.com/grpc/grpc

ARG PREFIX=/farm_ng/env
ARG PARALLEL=1
RUN set -ex && \
    mkdir -p build-grpc && cd build-grpc && \
    cmake \
    -DCMAKE_INSTALL_PREFIX=/install_grpc \
    -DCMAKE_PREFIX_PATH=$PREFIX \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    ../grpc && \
    cmake --build . --parallel $PARALLEL --target install --config Release


FROM scratch
ARG PREFIX=/farm_ng/env
COPY --from=0 /install_grpc $PREFIX
