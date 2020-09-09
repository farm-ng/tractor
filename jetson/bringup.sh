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

# add some routes for wlan0, lo
route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
route add -net 224.0.0.0 netmask 240.0.0.0 dev wlan0

# enable multicast on loopback, cause we may not have a wireless or wired link.
ifconfig lo multicast
ifconfig wlan0 multicast

# Power cycle the usb bus, due to an issue with the t265 camera
# bootloader having a race condition if power is supplied before the
# usb is ready.  uhubctl can power cycle the nano usb bus power in a
# way that seems to work around this issue.
#
# https://github.com/IntelRealSense/librealsense/issues/4681
# https://github.com/mvp/uhubctl/issues/258
/opt/farm_ng/sbin/uhubctl -l 2-1 -p 1 -a cycle

# let the usb devices come back, TODO(ethanrublee) watch for
# events?
sleep 1

# bring up canbus
if [ ! -f "/opt/farm_ng/flags/canbus_disable.touch" ]; then
    $SERVICE_DIR/bringup_can.sh
fi

touch /tmp/tractor-ready.touch

while true
do
    if [ ! -f "/opt/farm_ng/flags/canbus_disable.touch" ]; then
        ip -details -statistics link show can0
    fi
    sleep 5
done
