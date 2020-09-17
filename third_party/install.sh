#!/bin/bash

. ../setup.bash

cd $FARM_NG_ROOT/third_party
mkdir -p build-ceres
cd build-ceres
cmake -DCMAKE_INSTALL_PREFIX=$FARM_NG_ROOT/env -DCMAKE_PREFIX_PATH=$FARM_NG_ROOT/env/ -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF ../ceres-solver
make -j$(nproc --ignore=1)
make install

cd $FARM_NG_ROOT/third_party
mkdir -p build-sophus
cd build-sophus
cmake -DCMAKE_INSTALL_PREFIX=$FARM_NG_ROOT/env -DCMAKE_PREFIX_PATH=$FARM_NG_ROOT/env/ -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF ../Sophus
make -j$(nproc --ignore=1)
make install

cd $FARM_NG_ROOT/third_party
mkdir -p build-apriltag
cd build-apriltag
cmake -DCMAKE_INSTALL_PREFIX=$FARM_NG_ROOT/env -DCMAKE_PREFIX_PATH=$FARM_NG_ROOT/env/ -DCMAKE_BUILD_TYPE=Release ../apriltag
make -j$(nproc --ignore=1)
make install
