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
    wget \
    && \
    apt-get clean


RUN pip3 install --upgrade pip && pip3 install cmake
RUN wget https://github.com/opencv/opencv/archive/4.5.1.tar.gz && tar -xvzf 4.5.1.tar.gz

RUN apt-get update --fix-missing && \
   apt-get install -y  --no-install-recommends \
   libavcodec-dev \
   libavformat-dev \
   libavresample-dev \
   libavutil-dev \
   libgstreamer-plugins-base1.0-dev \
   libgstreamer1.0-dev \
   libswscale-dev \
   libv4l-dev \
   libx264-dev \
   libxvidcore-dev \
   python3-dev \
   python3-numpy \
   && \
   apt-get clean

RUN pip3 install \
 cython \
 numpy

ARG PREFIX=/farm_ng/env
ARG PARALLEL=1
RUN set -ex && \
    mkdir -p build && cd build && \
    cmake \
    -DCMAKE_INSTALL_PREFIX=/opencv_install \
    -DCMAKE_PREFIX_PATH=${PREFIX} \
    -DCMAKE_BUILD_TYPE=Release \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D INSTALL_C_EXAMPLES=OFF \
    -D PYTHON2_EXECUTABLE=/usr/bin/python2.7 \
    -D BUILD_opencv_python2=OFF \
    -D PYTHON3_EXECUTABLE=$(which python3) \
    -D WITH_GSTREAMER=ON \
    -D BUILD_EXAMPLES=OFF \
    -D BUILD_PROTOBUF=OFF \
    -D BUILD_TESTS=OFF \
    -DBUILD_LIST=calib3d,videoio,imgproc,highgui,video,python3 \
    ../opencv-4.5.1 && \
    cmake --build . --parallel $PARALLEL --target install --config Release


FROM scratch as opencv_installed
ARG PREFIX=/farm_ng/env
COPY --from=0 /opencv_install $PREFIX
