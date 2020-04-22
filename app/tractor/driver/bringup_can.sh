#!/bin/bash -ex
busybox devmem 0x0c303000
busybox devmem 0x0c303008
busybox devmem 0x0c303010
busybox devmem 0x0c303018

busybox devmem 0x0c303000 32 0x0000C400
busybox devmem 0x0c303008 32 0x0000C458
busybox devmem 0x0c303010 32 0x0000C400
busybox devmem 0x0c303018 32 0x0000C458

busybox devmem 0x0c303000
busybox devmem 0x0c303008
busybox devmem 0x0c303010
busybox devmem 0x0c303018

modprobe can
modprobe can-raw
modprobe mttcan

ip link set can0 down
ip link set can0 type can bitrate 1000000 berr-reporting off one-shot off restart-ms 5
ip link set can0 up

ip -details -statistics link show can0

