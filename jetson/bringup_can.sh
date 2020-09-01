#!/bin/bash -ex
ip link set can0 down || true
until ip link set can0 up txqueuelen 1000 type can bitrate 500000
do
     sleep 1
done
ip -details -statistics link show can0
