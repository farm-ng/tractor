#!/bin/bash -ex
if [ -z ${FARM_NG_ROOT} ]; then
    echo "Please source <FARM_NG_ROOT>/setup.bash"
    exit -1
fi

sudo $FARM_NG_ROOT/app/tractor/driver/bringup_can.sh
python3 -m farm_ng.tractor.driver
