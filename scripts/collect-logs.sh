#!/bin/bash

LOGNAME="farm-ng-log-$(hostname)-$(date --iso-8601=seconds)"
LOGDIR=`mktemp -d -t $LOGNAME-XXX`

systemctl list-unit-files | grep "tractor" | awk '{print $1}' | while IFS= read -r unit; do
  logfile=$LOGDIR/$unit.log
  echo "Writing to $logfile"
  journalctl -b --output=short-iso-precise --no-pager -u $unit > $logfile
done

logfile=$LOGDIR/systemd.log
echo "Writing to $logfile"
systemctl status --no-pager 'tractor*' > $logfile

logfile=$LOGDIR/dmesg.log
echo "Writing to $logfile"
journalctl -b --dmesg --no-pager > $logfile

zipfile=$LOGDIR.zip
echo "Zipping to $zipfile"
zip -r $zipfile $LOGDIR

cp $zipfile $HOME/tractor-logs/adhoc/$LOGNAME.zip
cd $HOME/tractor-logs
git add adhoc/$LOGNAME.zip
git commit -m "[$(hostname)] automated upload"
git push origin master
cd -
