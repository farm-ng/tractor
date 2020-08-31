system configuration for nano
-----------------------------

This directory has scripts to install systemd level services to make
the tractor functional at boot.

Ensure the repository is up to date (including submodules), built.

This assumes the user on the nano is farmer and the code is located at ``~/tractor``


Run the jetson/install.sh script to install the services.
```bash
cd ~/tractor/jetson
sudo ./install.sh
```

This installs the following services:

- tractor-bringup.service - responsible for power cycling the USB bus
  and starting the canbus interface.

- tractor-steering.service - reads from a bluetooth PS4 joystick and
  publishes steering commands

- tractor.service - Responsible for talking to the motors, and
  receiving steering commands to drive the tractor arround.


The install script enables these services so that they start at boot.

To see their log output:

```bash
journalctl -f -u tractor
journalctl -f -u tractor-bringup
journalctl -f -u tractor-steering
```

Or ``tail -f /var/log/syslog`` ...

