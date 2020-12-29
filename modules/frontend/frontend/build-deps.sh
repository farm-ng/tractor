#!/bin/bash

for dir in $FARM_NG_ROOT/modules/*
do
  # Module name from path (https://stackoverflow.com/questions/9011233)
  module="${dir##*/}"
  cd $FARM_NG_ROOT/build/modules/$module/protos/ts && yarn
done
