#!/bin/bash

for module in core calibration frontend motors perception tractor
do
  cd $FARM_NG_ROOT/build/modules/$module/protos/ts && yarn
done
