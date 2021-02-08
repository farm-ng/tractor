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
    python3-dev \
    python3-numpy \
    wget \
    && \
    apt-get clean


RUN pip3 install --upgrade pip && pip3 install cmake
RUN wget https://github.com/AprilRobotics/apriltag/archive/v3.1.3.tar.gz && tar -xvzf v3.1.3.tar.gz

ARG PREFIX=/farm_ng/env
ARG PARALLEL=1
RUN set -ex && \
    mkdir -p build && cd build && \
    mkdir -p /root/.local/lib/python3.6/site-packages  && \
    cmake \
    -DCMAKE_INSTALL_PREFIX=/apriltag_install \
    -DCMAKE_PREFIX_PATH=${PREFIX} \
    -DCMAKE_BUILD_TYPE=Release \
    ../apriltag-3.1.3 && \
    cmake --build . --parallel $PARALLEL --target install --config Release && \
    cp -r /root/.local/lib/python3.6 /apriltag_install/lib/


FROM scratch as apriltag_installed
ARG PREFIX=/farm_ng/env
COPY --from=0 /apriltag_install $PREFIX
