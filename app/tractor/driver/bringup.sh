#!/bin/bash -ex
app/tractor/driver/bringup_can.sh
app/tractor/driver/bringup_bluetooth.sh
while true
do
    ip -details -statistics link show can0
    sleep 5
done

