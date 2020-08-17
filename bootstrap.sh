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

sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u

# install system deps, punt for now.
sudo apt-get update
sudo apt-get install apt-utils -y
sudo apt-get install -y \
     build-essential \
     cmake \
     libboost-filesystem-dev \
     libboost-regex-dev \
     libboost-system-dev \
     libprotobuf-dev \
     librealsense2-dev \
     librealsense2-utils \
     protobuf-compiler \
     python3-pip

sudo pip3 install virtualenv

if [ ! -f $FARM_NG_ROOT/env/bin/activate ]; then
    virtualenv $FARM_NG_ROOT/env
fi

. $FARM_NG_ROOT/env/bin/activate
pip install -r $FARM_NG_ROOT/requirements.txt
