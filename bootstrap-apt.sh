#!/bin/bash -ex

# Realsense apt sources
if ! dpkg -s librealsense2-dev > /dev/null 2>&1; then
  sudo apt-get update && sudo apt-get install -y software-properties-common apt-utils gnupg2
  sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key
  sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
fi

# Yarn apt sources
if ! dpkg -s yarn > /dev/null 2>&1; then
  curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | sudo apt-key add -
  echo "deb https://dl.yarnpkg.com/debian/ stable main" | sudo tee /etc/apt/sources.list.d/yarn.list
fi

# Grafana apt sources
if ! dpkg -s grafana > /dev/null 2>&1; then
  wget -q -O - https://packages.grafana.com/gpg.key | sudo apt-key add -
  echo "deb https://packages.grafana.com/oss/deb stable main" | sudo tee -a /etc/apt/sources.list.d/grafana.list
fi

# Node apt sources
if ! dpkg -s nodejs > /dev/null 2>&1; then
  curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
fi

# System dependencies
sudo apt-get update
sudo apt-get install -y \
     apt-transport-https \
     build-essential \
     ca-certificates \
     clang \
     clang-10 \
     clang-tidy-10 \
     cmake \
     curl \
     dirmngr \
     git \
     git-lfs \
     grafana \
     gstreamer1.0-libav \
     libatlas-base-dev \
     libboost-filesystem-dev \
     libboost-regex-dev \
     libboost-system-dev \
     libclang-10-dev \
     libeigen3-dev \
     libgoogle-glog-dev \
     libgstreamer-plugins-base1.0-dev \
     libgstreamer1.0-dev \
     libprotobuf-dev \
     librealsense2-dev \
     librealsense2-utils \
     libsuitesparse-dev \
     libusb-1.0-0-dev \
     lsb-release \
     network-manager \
     nodejs \
     protobuf-compiler \
     python3-pip \
     yarn

sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-10 100
sudo update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-10 100
sudo update-alternatives --install /usr/bin/run-clang-tidy run-clang-tidy /usr/bin/run-clang-tidy-10 100
