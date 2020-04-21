#!/bin/bash -ex
find /usr/src/app/
if [ -f /etc/farm_ng/rtkrover/rtkrcv.conf ]; then
   cat  /etc/farm_ng/rtkrover/rtkrcv.conf
else
   mkdir -p  /etc/farm_ng/rtkrover/
   cp /usr/src/app/etc/farm_ng/rtkrover/rtkrcv.conf  /etc/farm_ng/rtkrover/
fi 


if [ -f /etc/farm_ng/rtkrover/m8t_5hz_usb.cmd ]; then
   cat /etc/farm_ng/rtkrover/m8t_5hz_usb.cmd
else
   mkdir -p  /etc/farm_ng/rtkrover/
   cp /usr/src/app/etc/farm_ng/rtkrover/m8t_5hz_usb.cmd  /etc/farm_ng/rtkrover/
fi 

rtkrcv -p 2023 -m 2024 -s -o /etc/farm_ng/rtkrover/rtkrcv.conf
