#!/bin/bash -ex

. /farm_ng/setup.bash

# Build first, so other deps can build against grpc-installed protobuf
cd $FARM_NG_ROOT/third_party
mkdir -p build-grpc
cd build-grpc
cmake -DCMAKE_INSTALL_PREFIX=$FARM_NG_ROOT/env -DCMAKE_PREFIX_PATH=$FARM_NG_ROOT/env/ -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON ../grpc
make -j$(nproc --ignore=1)
make install

cd $FARM_NG_ROOT/third_party
mkdir -p build-ceres
cd build-ceres
cmake -DCMAKE_INSTALL_PREFIX=$FARM_NG_ROOT/env -DCMAKE_PREFIX_PATH=$FARM_NG_ROOT/env/ -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_SHARED_LIBS=ON ../ceres-solver
make -j$(nproc --ignore=1)
make install

cd $FARM_NG_ROOT/third_party
mkdir -p build-sophus
cd build-sophus
cmake -DCMAKE_INSTALL_PREFIX=$FARM_NG_ROOT/env -DCMAKE_PREFIX_PATH=$FARM_NG_ROOT/env/ -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF ../Sophus
make -j$(nproc --ignore=1)
make install

cd $FARM_NG_ROOT/third_party
mkdir -p build-apriltag
cd build-apriltag
cmake -DCMAKE_INSTALL_PREFIX=$FARM_NG_ROOT/env -DCMAKE_PREFIX_PATH=$FARM_NG_ROOT/env/ -DCMAKE_BUILD_TYPE=Release ../apriltag
make -j$(nproc --ignore=1)
make install
