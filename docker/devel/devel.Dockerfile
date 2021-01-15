ARG base_image=ubuntu:18.04
ARG grpc_tag=farmng/build-grpc:latest
ARG opencv_tag=farmng/build-opencv:latest
ARG ceres_tag=farmng/build-ceres:latest
ARG apriltag_tag=farmng/build-apriltag:latest
ARG sophus_tag=farmng/build-sophus:latest


FROM $grpc_tag AS grpc
FROM $opencv_tag AS opencv
FROM $ceres_tag AS ceres
FROM $apriltag_tag AS apriltag
FROM $sophus_tag AS sophus


FROM $base_image AS devel

RUN apt-get update --fix-missing && \
    apt-get install -y --no-install-recommends \
    apt-transport-https \
    apt-utils \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    wget \
    ca-certificates \
    && \
    apt-get clean

RUN apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key && \
    add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u

RUN curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | apt-key add - && \
    echo "deb https://dl.yarnpkg.com/debian/ stable main" | tee /etc/apt/sources.list.d/yarn.list

RUN NODEREPO="node_12.x" && \
    DISTRO=$(lsb_release -c -s) && \
    curl -s https://deb.nodesource.com/gpgkey/nodesource.gpg.key | apt-key add - && \
    echo "deb https://deb.nodesource.com/${NODEREPO} ${DISTRO} main" > /etc/apt/sources.list.d/nodesource.list && \
    echo "deb-src https://deb.nodesource.com/${NODEREPO} ${DISTRO} main" >> /etc/apt/sources.list.d/nodesource.list

RUN apt-get update --fix-missing && \
    apt-get install -y --no-install-recommends \
    build-essential \
    clang \
    cython \
    dirmngr \
    doxygen \
    git \
    graphviz \
    gstreamer1.0-libav \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-tools \
    libboost-filesystem-dev \
    libboost-regex-dev \
    libboost-system-dev \
    libdbus-glib-1-dev \
    libeigen3-dev \
    libgoogle-glog-dev \
    librealsense2-dev \
    librealsense2-utils \
    libsuitesparse-dev \
    nodejs \
    network-manager \
    python3-dev \
    python3-pip \
    python3-numpy \
    sudo \
    yarn \
    && \
    apt-get clean

RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 10

ARG PREFIX=/farm_ng/env

ENV FARM_NG_PREFIX=$PREFIX
ENV PYTHONPATH=$FARM_NG_PREFIX/lib/python3.6/site-packages:$FARM_NG_PREFIX/lib/python3.6/dist-packages
ENV LD_LIBRARY_PATH=$FARM_NG_PREFIX/lib

ENV FARM_NG_GOPATH=$FARM_NG_PREFIX/go
ENV GOPATH=$FARM_NG_GOPATH:$GOPATH
ENV PATH=$FARM_NG_GOPATH/bin:$FARM_NG_PREFIX/bin:/usr/local/go/bin:$PATH

RUN arch=`dpkg --print-architecture` && \
    wget https://golang.org/dl/go1.15.1.linux-${arch}.tar.gz -P /tmp/ && \
    tar -C /usr/local -xzf /tmp/go1.15.1.linux-${arch}.tar.gz && \
    /usr/local/go/bin/go version

RUN nodejs --version && \
    npm install -g long ts-proto@^1.37.0

RUN FARM_NG_GOPATH=$PREFIX/go && \
    export GOPATH=$FARM_NG_GOPATH:$GOPATH && \
    export PATH=$FARM_NG_GOPATH/bin:$PREFIX/bin:/usr/local/go/bin:/usr/bin:$PATH && \
    go get -u github.com/golang/protobuf/protoc-gen-go && \
    go get -u github.com/twitchtv/twirp/protoc-gen-twirp

RUN python -m pip install --upgrade pip setuptools && python -m pip install \
    git+https://github.com/alex-eri/python-networkmanager.git@ec5d10ef7e18f27b24b439d888cea89c1f802f5c \
    git+https://github.com/utiasSTARS/liegroups.git@11e0203048def0345097cb42c664aa91435c3dd0 \
    breathe==4.25.1 \
    cmake==3.18.4.post1 \
    grpcio==1.34.0 \
    linuxfd==1.5 \
    protobuf==3.14.0 \
    sphinx-rtd-theme==0.5.0 \
    sphinx-tabs==1.3.0

# [docs] copy_third_party
COPY --from=grpc $PREFIX $PREFIX
COPY --from=opencv $PREFIX $PREFIX
COPY --from=ceres $PREFIX $PREFIX
COPY --from=sophus $PREFIX $PREFIX
COPY --from=apriltag $PREFIX $PREFIX
# [docs] copy_third_party

ARG WORKSPACE_DIR=/workspace/tractor
ENV FARM_NG_ROOT=$WORKSPACE_DIR
ENV GOPATH=$FARM_NG_ROOT/.go:$GOPATH

ENV PYTHONPATH=$FARM_NG_ROOT/modules/core/python:$FARM_NG_ROOT/build/modules/core/python/protos/python:$PYTHONPATH
ENV PYTHONPATH=$FARM_NG_ROOT/modules/perception/python:$FARM_NG_ROOT/build/modules/perception/python/protos/python:$PYTHONPATH
ENV PYTHONPATH=$FARM_NG_ROOT/modules/calibration/python:$FARM_NG_ROOT/build/modules/calibration/python/protos/python:$PYTHONPATH
ENV PYTHONPATH=$FARM_NG_ROOT/modules/frontend/python:$FARM_NG_ROOT/build/modules/frontend/python/protos/python:$PYTHONPATH
ENV PYTHONPATH=$FARM_NG_ROOT/modules/tractor/python:$FARM_NG_ROOT/build/modules/tractor/python/protos/python:$PYTHONPATH
