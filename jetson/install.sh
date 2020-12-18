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

# install uhubctl
mkdir -p /opt/farm_ng/systemd
prefix=/opt/farm_ng make -C $SERVICE_DIR/uhubctl
prefix=/opt/farm_ng make -C $SERVICE_DIR/uhubctl install

# install grafana configuration
# sudo rsync --verbose --recursive --delete --owner --group --chown=root:grafana $SERVICE_DIR/metrics/grafana/provisioning/ /etc/grafana/provisioning

# clean
systemctl list-unit-files | grep "tractor" | awk '{print $1}' | xargs --no-run-if-empty -n1 sudo systemctl disable
rm -f /opt/farm_ng/systemd/*.sh
rm -f /etc/systemd/system/multi-user.target.wants/tractor*
rm -f /etc/systemd/system/tractor*

# install
cp $SERVICE_DIR/*.sh /opt/farm_ng/systemd
cp $SERVICE_DIR/tractor-bringup.service /etc/systemd/system/
#cp $SERVICE_DIR/*.path /etc/systemd/system/

# add udev rule so we can have services wait on the usb bus
# https://superuser.com/a/1398400
cp $SERVICE_DIR/20-usb-bus.rules /etc/udev/rules.d/

# refresh
systemctl daemon-reload

# start automatically on boot
systemctl enable tractor-bringup.service
# systemctl enable tractor.service
# systemctl enable tractor.path
# systemctl enable tractor-steering.service
# systemctl enable tractor-steering.path
# systemctl enable tractor-camera.service
# systemctl enable tractor-camera.path
# systemctl enable tractor-webservices.service
# systemctl enable tractor-webservices.path
# systemctl enable tractor-logger.service
# systemctl enable tractor-logger.path
# systemctl enable tractor-programd.service
# systemctl enable tractor-programd.path
