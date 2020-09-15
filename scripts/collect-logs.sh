#!/bin/bash

LOGDIR=`mktemp -d -t farm-ng-log-$(date +"%s")-XXX`

systemctl list-unit-files | grep "tractor" | awk '{print $1}' | while IFS= read -r unit; do
  logfile=$LOGDIR/$unit.log
  echo "Writing to $logfile"
  journalctl -b --output=short-iso-precise --no-pager -u $unit > $logfile
done

logfile=$LOGDIR/systemd.log
echo "Writing to $logfile"
systemctl status --no-pager tractor* > $logfile

logfile=$LOGDIR/dmesg.log
echo "Writing to $logfile"
journalctl -b --dmesg --no-pager > $logfile

zipfile=$LOGDIR.zip
echo "Zipping to $zipfile"
zip -r $zipfile $LOGDIR
