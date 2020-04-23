# tractor app

## env.sh

env.sh makes it easy to setup the paths required on bare
metal to play with things. Its also used inside our dockers to prefix
commands so that ``python -m farm_ng.joystick`` actually works.

It starts a new bash shell, sourcing a custom bashrc found in the root
of the git repository ``$FARM_NG_ROOT/bashrc`` and sourcing
``$FARM_NG_ROOT/setup.bash``.

``FARM_NG_ROOT`` is exported by exported by ``setup.bash``, and points
to the root of the tractor git repository.

To enter bash subshell using env.sh:

```bash
farmer@tractor01:~$ ~/code/tractor/env.sh 
f-g:farmer@tractor01:~$ echo $FARM_NG_ROOT 
/home/farmer/code/tractor
f-g:farmer@tractor01:~$ cd $FARM_NG_ROOT
```
To exit the bash:

```bash
f-g:farmer@tractor01:~/code/tractor$ exit
exit
farmer@tractor01:~$
```


# app/tractor

The following assume you are inside ``tractor/env.sh``:

```bash
farmer@tractor01:~$ ~/code/tractor/env.sh
```

To build the tractor app on the tractor/xavier (TODO(rublee) make
workflow for building on a workstation with docker-compose or balena):

```bash
cd $FARM_NG_ROOT/tractor
docker-compose build
```

To bring up the tractor app:
```bash
cd $FARM_NG_ROOT/tractor
docker-compose -d up
```

Browse to ``http://<IP_of_tractor>`` to see the app.

To stop the app:
```bash
docker-compose down
```

To look at logs:
```
docker-compose logs
```

To look at just a single service's logs:
```
docker-compose logs rtkrover 
```

To develop the front end on the robot:
```bash
cd $FARM_NG_ROOT/
# stop the app
docker-compose down
# start rtkrover service, expose 2023 telnet port to host network
docker-compose -f docker-compose.yml -f apps/tractor/docker-compose.dev.yml up -d rtkrover
# start the frontend from terminal on the xavier host
python3 -m farm_ng.tractor.frontend
```