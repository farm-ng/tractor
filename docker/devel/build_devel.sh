#!/bin/bash -ex

./build_thirdparty.sh grpc
./build_thirdparty.sh opencv
./build_thirdparty.sh sophus
./build_thirdparty.sh apriltag
./build_thirdparty.sh ceres

docker build --memory=8024m --build-arg PARALLEL=$(nproc --ignore=2) -f devel.Dockerfile -t farmng/devel:latest -t farmng/devel:`git rev-parse --short HEAD` ./
