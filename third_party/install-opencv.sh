#!/bin/bash -ex
. /farm_ng/setup.bash

mkdir -p $FARM_NG_ROOT/third_party/build-opencv
cd $FARM_NG_ROOT/third_party/build-opencv

cmake \
    -DCMAKE_INSTALL_PREFIX=$FARM_NG_ROOT/env \
    -DCMAKE_PREFIX_PATH=$FARM_NG_ROOT/env/ \
    -DCMAKE_BUILD_TYPE=Release \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D INSTALL_C_EXAMPLES=OFF \
    -D PYTHON_EXECUTABLE=/usr/bin/python2.7 \
    -D BUILD_opencv_python2=OFF \
    -D PYTHON3_EXECUTABLE=$(which python) \
    -D WITH_GSTREAMER=ON \
    -D BUILD_EXAMPLES=OFF \
    -D BUILD_PROTOBUF=OFF \
    -D BUILD_TESTS=OFF \
    -DBUILD_LIST=calib3d,videoio,imgproc,highgui,video,python3 \
    ../opencv

make -j$(nproc --ignore=1)
make install
