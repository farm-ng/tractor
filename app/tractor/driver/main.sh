#!/bin/bash -ex
. setup.bash
. $FARM_NG_ROOT/env/bin/activate
python -m farm_ng.tractor.driver
