#!/bin/bash -ex
#echo 1 > /sys/module/bluetooth/parameters/disable_ertm
#cat /sys/module/bluetooth/parameters/disable_ertm
rfkill list all
rfkill unblock bluetooth
rfkill list all
