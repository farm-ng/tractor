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

$SERVICE_DIR/bringup_can.sh
while true
do
    ip -details -statistics link show can0
    sleep 5
done
