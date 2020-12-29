#!/bin/bash -ex
. setup.bash
sleep 5 # wait for camera to be ready
build/modules/perception/cpp/farm_ng/camera_pipeline --jetson
