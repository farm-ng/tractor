FROM ubuntu:18.04

WORKDIR /farm_ng
RUN export FARM_NG_ROOT=/farm_ng

# Install system dependencies
RUN apt-get update --fix-missing && \
  apt-get install -y --no-install-recommends \
  apt-utils \
  build-essential \
  cmake \
  curl \
  gstreamer1.0-libav \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-tools \
  gnupg2 \
  libboost-filesystem-dev \
  libboost-regex-dev \
  libboost-system-dev \
  libdbus-glib-1-dev \
  libeigen3-dev \
  libgoogle-glog-dev \
  libgstreamer-plugins-base1.0-dev \
  libgstreamer1.0-dev \
  libprotobuf-dev \
  libsuitesparse-dev \
  protobuf-compiler \
  python3-dev \
  python3-pip \
  software-properties-common \
  git \
  && apt-get clean

# Install Realsense drivers
RUN apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key
RUN add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
RUN apt-get update && apt-get install -y --no-install-recommends librealsense2-dev librealsense2-utils

# Install minimum python configuration (required to build some third_party C++ libraries)
# TODO(isherman): Should this really be necessary?
COPY setup.bash ./
RUN pip3 install virtualenv
RUN virtualenv ./env
RUN . ./env/bin/activate

# Build third-party code
COPY third_party third_party
RUN cd third_party && ./install.sh

# Build first-party code
COPY CMakeLists.txt .
COPY cpp cpp
COPY protos protos
RUN	mkdir -p build && \
  cd build && \
  cmake -DCMAKE_PREFIX_PATH=`pwd`/../env -DCMAKE_BUILD_TYPE=Release .. && \
  make -j`nproc --ignore=1`

# TODO(isherman): Reduce size of final image with multi-stage build
# https://devblogs.microsoft.com/cppblog/using-multi-stage-containers-for-c-development/
