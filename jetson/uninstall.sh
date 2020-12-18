#!/bin/bash -ex
# clean
systemctl list-unit-files | grep "tractor" | awk '{print $1}' | xargs --no-run-if-empty -n1 sudo systemctl disable
rm -f /opt/farm_ng/systemd/*.sh
rm -f /etc/systemd/system/multi-user.target.wants/tractor*
rm -f /etc/systemd/system/tractor*
