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

# clean up old systemctl services
rm -f /etc/systemd/system/multi-user.target.wants/tractor*
rm -f /etc/systemd/system/tractor*

mkdir -p /opt/farm_ng/systemd
prefix=/opt/farm_ng make -C $SERVICE_DIR/uhubctl
prefix=/opt/farm_ng make -C $SERVICE_DIR/uhubctl install
cp $SERVICE_DIR/*.sh /opt/farm_ng/systemd
cp $SERVICE_DIR/*.service /etc/systemd/system/
cp $SERVICE_DIR/*.path /etc/systemd/system/
# https://superuser.com/a/1398400 - add udev rule so we can have services wait on the usb bus.
cp $SERVICE_DIR/20-usb-bus.rules /etc/udev/rules.d/
systemctl daemon-reload

# start on boot always...
systemctl enable tractor-bringup.service
systemctl enable tractor.path
systemctl enable tractor-steering.path
systemctl enable tractor-camera.path
systemctl enable tractor-webservices.path

systemctl start tractor.path
systemctl start tractor-steering.path
systemctl start tractor-camera.path
systemctl start tractor-webservices.path

