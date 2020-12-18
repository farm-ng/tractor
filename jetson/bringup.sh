#!/bin/bash -ex
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  # if $SOURCE was a relative symlink, we need to resolve it
  # relative to the path where the symlink file was located
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
done
export SERVICE_DIR=$( cd "$( dirname "${SOURCE}" )" >/dev/null 2>&1 && pwd )

# add multicast route for lo
EXIST=`ip route show 224.0.0.0/4  | wc -l`
if [ $EXIST -eq 0 ]
then
    route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
fi
# if [ $EXIST -eq 1 ]
# then
#     route add -net 224.0.0.0 netmask 240.0.0.0 dev wlan0
# fi


# enable multicast on loopback, cause we may not have a wireless or wired link.
ifconfig lo multicast
# ifconfig wlan0 multicast

# Power cycle the usb bus, due to an issue with the t265 camera
# bootloader having a race condition if power is supplied before the
# usb is ready.  uhubctl can power cycle the nano usb bus power in a
# way that seems to work around this issue.
#
# https://github.com/IntelRealSense/librealsense/issues/4681
# https://github.com/mvp/uhubctl/issues/258
#
# Also seems important to cycle the power for the canable pro
# device. Sometimes after a soft reboot we've noticed the canbus
# doesn't start
/opt/farm_ng/sbin/uhubctl -l 2-1 -p 1 -a cycle

# let the usb devices come back, TODO(ethanrublee) watch for
# events?
sleep 1

$SERVICE_DIR/bringup_can.sh
touch /tmp/tractor-ready.touch

while true
do
    ip -details -statistics link show can0
    sleep 5
done
