ARG base_image=ubuntu:18.04
FROM $base_image
RUN apt-get update --fix-missing && \
    apt-get install -y --no-install-recommends \
    autoconf \
    build-essential \
    ca-certificates \
    libtool \
    pkg-config \
    python3-pip \
    libeigen3-dev \
    wget \
    && \
    apt-get clean

RUN pip3 install --upgrade pip && pip3 install cmake
RUN wget https://github.com/strasdat/Sophus/archive/a0fe89a323e20c42d3cecb590937eb7a06b8343a.tar.gz && tar -xvzf a0fe89a323e20c42d3cecb590937eb7a06b8343a.tar.gz

ARG PREFIX=/farm_ng/env
ARG PARALLEL=1
RUN set -ex && \
    mkdir -p build && cd build && \
    cmake \
    -DCMAKE_INSTALL_PREFIX=/sophus_install \
    -DCMAKE_PREFIX_PATH=${PREFIX} \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    ../Sophus-a0fe89a323e20c42d3cecb590937eb7a06b8343a && \
    cmake --build . --parallel $PARALLEL --target install --config Release


FROM scratch
ARG PREFIX=/farm_ng/env
COPY --from=0 /sophus_install $PREFIX
