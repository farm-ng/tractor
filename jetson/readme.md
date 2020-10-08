## system configuration for nano

This directory has scripts to install systemd level services to make
the tractor functional at boot.

Ensure the repository is up to date (including submodules), built.

This assumes the user on the nano is farmer and the code is located at `~/tractor`

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

- tractor-webservices.service - Responsible for proxying ipc traffic over webRTC,
  serving a Twirp API, and serving the static frontend.

- tractor-logger.service - Logs eventbus traffic.

- tractor-programd.service - Supervises "programs".

The install script enables these services so that they start at boot.

To see their log output:

```bash
journalctl -f -u tractor
journalctl -f -u tractor-bringup
journalctl -f -u tractor-steering
journalctl -f -u tractor-webservices
journalctl -f -u tractor-logger
journalctl -f -u tractor-programd
```

Or `tail -f /var/log/syslog` ...

## Debugging

```
systemd-analyze plot > bootup.svg
systemd-analyze dot --from-pattern='tractor*.service' | dot -Tsvg > graph.svg
systemd-analyze verify multi-user.target (look for anything tractor related)
systemctl list-unit-files
systemctl show -p Requires,Wants,Requisite,BindsTo,PartOf,Before,After <name>.service

```

## Wifi Configuration

### Enable the tractor as an access point

- Run `python -m farm_ng.wifi ap`.
- **NOTE**: If you are logged in via SSH this will terminate your SSH session
- You will be prompted to enter a password to use for this access point
- You can now connect your computer or mobile device to the `farm_ng-<tractorhostname>` wifi network.
- Your device will be assigned an address in the `192.168.48/24` subnet.
- The tractor is reachable at `192.168.48.1`.

### Change the tractor's access point password

- Run `python -m farm_ng.wifi list`
- Run `python -m farm_ng.wifi delete <id>` with the `farm_ng-<tractorhostname>` id from the previous step
- Run `python -m farm_ng.wifi ap`

### Connect the tractor to a wifi network

- NOTE: This workflow supports connecting to WiFi networks for which you already have a password saved, and for which you've enabled "All users may connect to this network". To connect to a wifi network for which you don't already have a password saved, please use the Gnome desktop tools or [nmcli](https://developer.gnome.org/NetworkManager/stable/nmcli.html).
- To list the WiFi networks for which you already have a password saved, run `python -m farm_ng.wifi list`
- To connect to a wifi network, run `python -m farm_ng.wifi connect <id>` with an `id` from the previous step.
