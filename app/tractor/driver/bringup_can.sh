#!/bin/bash -ex
ip link set can0 down
ip link set can0 up txqueuelen 1000 type can bitrate 500000

ip -details -statistics link show can0
