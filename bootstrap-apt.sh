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

# System dependencies
sudo apt-get update
sudo apt-get install -y \
     apt-transport-https \
     build-essential \
     ca-certificates \
     cmake \
     curl \
     dirmngr \
     git \
     git-lfs \
     libatlas-base-dev \
     libboost-filesystem-dev \
     libboost-regex-dev \
     libboost-system-dev \
     libeigen3-dev \
     libgoogle-glog-dev \
     libopencv-dev \
     libprotobuf-dev \
     librealsense2-dev \
     librealsense2-utils \
     libsuitesparse-dev \
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
if ! /usr/local/go/bin/go version | grep 1.15.1; then
  wget https://golang.org/dl/go1.15.1.linux-arm64.tar.gz -P /tmp/
  sudo tar -C /usr/local -xzf /tmp/go1.15.1.linux-arm64.tar.gz
  /usr/local/go/bin/go version
fi

# Node
if ! nodejs --version | grep 12.18.3; then
  curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
  sudo apt -y install nodejs
  nodejs --version
fi

# tractor-logs
if [ ! -d "$HOME/tractor-logs" ]; then
  if git clone git@github.com:farm-ng/tractor-logs.git $HOME/tractor-logs; then
    cd ~/tractor-logs
    git lfs install --local
    git config user.email `hostname`
    git config user.name `hostname`
  else
    echo "Please generate an SSH keypair and add it to the tractor-logs repository."
    exit 1
  fi
fi
