#!/bin/bash -ex
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  # if $SOURCE was a relative symlink, we need to resolve it
  # relative to the path where the symlink file was located
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
done
export FARM_NG_ROOT=$( cd "$( dirname "${SOURCE}" )" >/dev/null 2>&1 && pwd )

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

# System dependencies
sudo apt-get update
sudo apt-get install -y \
     apt-transport-https \
     build-essential \
     ca-certificates \
     clang \
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
     protobuf-compiler \
     python3-pip \
     yarn

# Virtualenv
if ! pip3 show virtualenv > /dev/null 2>&1; then
  pip3 install virtualenv
fi

# Go
arch=`dpkg --print-architecture`
if ! /usr/local/go/bin/go version | grep 1.15.1; then
  wget https://golang.org/dl/go1.15.1.linux-${arch}.tar.gz -P /tmp/
  sudo tar -C /usr/local -xzf /tmp/go1.15.1.linux-${arch}.tar.gz
  /usr/local/go/bin/go version
fi

# Node
if ! nodejs --version | grep 12.18.3; then
  curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
  sudo apt -y install nodejs
  nodejs --version
fi

# Prometheus Node Exporter
if ! node_exporter --version | grep 1.0.1; then
  wget https://github.com/prometheus/node_exporter/releases/download/v1.0.1/node_exporter-1.0.1.linux-${arch}.tar.gz -P /tmp/
  tar -C /tmp -xzf /tmp/node_exporter-1.0.1.linux-${arch}.tar.gz
  sudo cp /tmp/node_exporter-1.0.1.linux-${arch}/node_exporter /usr/local/bin
fi

# Prometheus
if ! prometheus --version | grep 2.21.0; then
  wget https://github.com/prometheus/prometheus/releases/download/v2.21.0/prometheus-2.21.0.linux-${arch}.tar.gz -P /tmp/
  tar -C /tmp -xzf /tmp/prometheus-2.21.0.linux-${arch}.tar.gz
  sudo cp /tmp/prometheus-2.21.0.linux-${arch}/prometheus /usr/local/bin
fi

# TODO: @jin this requires special permission
# # tractor-logs
# if [ ! -d "$HOME/tractor-logs" ]; then
#   if git clone git@github.com:farm-ng/tractor-logs.git $HOME/tractor-logs; then
#     cd ~/tractor-logs
#     git lfs install --local
#     git config user.email `hostname`
#     git config user.name `hostname`
#   else
#     echo "Please generate an SSH keypair and add it to the tractor-logs repository."
#     exit 1
#   fi
# fi
