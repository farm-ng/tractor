ARG base_image=ubuntu:18.04
FROM $base_image
RUN apt-get update --fix-missing && \
    apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    libatlas-base-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libsuitesparse-dev \
    python3-pip \
    wget \
    && \
    apt-get clean

RUN pip3 install --upgrade pip && pip3 install cmake
RUN wget  https://github.com/ceres-solver/ceres-solver/archive/2.0.0.tar.gz && tar -xvzf 2.0.0.tar.gz

ARG PREFIX=/farm_ng/env
ARG PARALLEL=1
RUN set -ex && \
    mkdir -p build && cd build && \
    cmake \
    -DCMAKE_INSTALL_PREFIX=/ceres_install \
    -DCMAKE_PREFIX_PATH=${PREFIX} \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_SHARED_LIBS=ON \
    ../ceres-solver-2.0.0 && \
    cmake --build . --parallel $PARALLEL --target install --config Release


FROM scratch
ARG PREFIX=/farm_ng/env
COPY --from=0 /ceres_install $PREFIX
