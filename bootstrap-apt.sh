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

apt-get update && apt-get install -y software-properties-common apt-utils gnupg2

apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key
add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u

# install system deps, punt for now.
apt-get update
apt-get install -y \
     build-essential \
     cmake \
     git \
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
     network-manager \
     protobuf-compiler \
     python3-pip

# Virtualenv
pip3 install virtualenv

# Go
wget https://golang.org/dl/go1.15.1.linux-arm64.tar.gz -P /tmp/
sudo tar -C /usr/local -xzf /tmp/go1.15.1.linux-arm64.tar.gz
/usr/local/go/bin/go version

# Node
sudo apt update
sudo apt -y install curl dirmngr apt-transport-https lsb-release ca-certificates
curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
sudo apt -y install nodejs
nodejs --version

# Yarn
curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | sudo apt-key add -
echo "deb https://dl.yarnpkg.com/debian/ stable main" | sudo tee /etc/apt/sources.list.d/yarn.list
sudo apt update
sudo apt -y install yarn
