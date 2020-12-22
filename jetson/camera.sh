#!/bin/bash -ex
. setup.bash
sleep 5 # wait for camera to be ready
build/cpp/farm_ng/perception/camera_pipeline --jetson
