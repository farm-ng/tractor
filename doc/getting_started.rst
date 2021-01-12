.. _chapter-getting_started:

===============
Getting Started
===============

farm-ng uses a Docker-based workflow for development and release packaging.

.. _getting_started-docker:

Docker
======

The following images are available on Dockerhub:

- `farmng/base <https://hub.docker.com/repository/docker/farmng/base>`_: All C++ and Python binaries and all source code.

- `farmng/webservices <https://hub.docker.com/repository/docker/farmng/webservices>`_: The web server and web application, but no source code.

- `farmng/devel <https://hub.docker.com/repository/docker/farmng/devel>`_: All source code and a base development environment with dependencies pre-installed.

Docker tags are used as follows:

- ``edge``: Built by CI for the latest commit on ``master``.

- ``latest``: Built by CI for the latest `Github release <https://github.com/farm-ng/tractor/releases>`_

- ``<x.y.z>``: Built by CI for the `Github release <https://github.com/farm-ng/tractor/releases>`_ ``x.y.z``

- ``<sha>``: Built ad hoc for a specific git commit.

Examples
--------
TODO

Setting up your Development Environment
=======================================
We have chosen to encapsulate the development environment in a docker container and to fully support
development in Visual Studio Code, to ease on boarding of collaborators with various levels of
experience, and simplify reproducibility.  Its also possible to build on bare metal, and the
Dockerfiles serve as documentation for our system dependencies.

We've tested this using development machines running Ubuntu 18.04 Desktop, NVidia's jetson nano and xavier platforms, and Mac OS.
Windows support should also be possible easily, but hasn't been tested.

To get started, you'll need a few system tools installed:

- git
- vscode
- docker
- docker-compose

Install vscode by following the directions here: https://code.visualstudio.com/Download

Install docker:

.. tabs::

   .. group-tab:: Ubuntu

      Ubuntu docker installation instructions are here: https://docs.docker.com/engine/install/ubuntu/

      Please ensure that you have added your user to the docker group, and log out and back in so that don't need root to run docker commands:

      .. code-block:: bash

        sudo usermod -aG docker $USER


   .. group-tab:: Mac OSX

      TODO

   .. group-tab:: Windows

      TODO

Install git

.. tabs::

   .. group-tab:: Ubuntu

      .. code-block:: bash

        sudo apt install git


   .. group-tab:: Mac OSX

      TODO

   .. group-tab:: Windows

      TODO

Install docker-compose

.. tabs::

   .. group-tab:: Ubuntu

      Using pip:

      .. code-block:: bash

        sudo apt install pip3
        sudo pip3 install docker-compose


   .. group-tab:: Mac OSX

      TODO

   .. group-tab:: Windows

      TODO


Get the code
------------

.. code-block:: bash

       git clone https://github.com/farm-ng/tractor.git

.. note::

  ``//<path>`` refers to a path relative to root of the repository wherever it is
  on your disk, e.g. ``/home/farmer/tractor``, ``~/code/tractor``.
  And most of the instructions assume you're starting in the root of the repository.

start the development container
-------------------------------

``docker-compose`` can be used to start the development docker container.
It mounts the repository on from your host as a volume, and it can be used to change the blobstore volume and set environment variables,
and then you can attach to it either through the command line or in vscode.

.. code-block:: bash

  cd docker
  docker-compose -f docker-compose.devel.yml up


You should see something similar to:

.. code-block::

  $ docker-compose -f docker-compose.devel.yml up
  Creating docker_devel_1 ... done
  Attaching to docker_devel_1
  devel_1  | farm-ng's devel container is running.  Press Ctrl-C to stop it.
  devel_1  | You can open up a shell inside the container:
  devel_1  |   docker exec -it docker_devel_1 bash


To stop the container, just press ``Ctrl-C``.

Now the docker container is up and running.  You can exec into it, interactively, and open as many shells as you like by the following:

.. code-block::

   docker exec -it docker_devel_1 bash


Try building configuring the cmake build inside the container

.. code-block:: bash

   docker exec docker_devel_1 bash -c ". setup.bash && mkdir -p build && cd build && cmake -DCMAKE_PREFIX_PATH=/farm_ng/env -DBUILD_DOCS=True .."


And you should see something similar to::

   -- The C compiler identification is GNU 7.5.0
   ...
   -- Configuring done
   -- Generating done
   -- Build files have been written to: /workspace/tractor/build


Now that the code is configured, you should see on the host system a build directory with a ``Makefile`` and ``CMakeCache.txt``

Let's try building these docs::

   $ docker exec docker_devel_1 /workspace/tractor/env.sh make -C build docs


And run a server to host then::

   $ docker exec docker_devel_1 bash -c "cd build/doc/sphinx && python3 -m http.server 8000"

Try browsing to http://localhost:8000/getting_started.html#start-the-development-container

Here is the docker-compose file, for reference.

.. literalinclude:: ../docker/docker-compose.devel.yml
   :language: yaml

We mount a volume at ``/root/``, as this is the root user's home directory in our docker,
so that bash history and other cached variables are persisted.

.. note::

   Currently the docker container user is root.  We want to fix that shortly, so it matches your user id on the host...
   Living on the edge here. See https://code.visualstudio.com/docs/remote/containers-advanced#_adding-a-nonroot-user-to-your-dev-container



vscode setup
------------

Now that you're comfortable execing into the development container let's boot up vscode.

Start vscode in the normal way and install the remote-containers extension: https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers

Launch VS Code Quick Open (Ctrl+P), paste the following command, and press enter ::

   ext install ms-vscode-remote.remote-containers

Now attach to the container ``docker_devel_1`` using **Remote-Containers: Attach to Running Container**.
For more information see https://code.visualstudio.com/docs/remote/attach-container


Now that you're attached, open the workspace file ``/workspace/tractor/workspace.code-workspace`` and install the suggested plugins.

Wait while vscode installs the extensions.

Try building with cmake in vscode by pressing ``F7``.
- Click Unspecified when prompted to Select a Kit.
- You can see the build output by clicking around in the UI.

vscode over ssh
---------------

This same workflow should also work if you're connected to a remote vscode host.
This is how we develop on remote machines such as robots.

To attach to a remote container over ssh just add the following to your settings.json file, and reopen vscode:

.. code-block::

   "docker.host":"ssh://your-remote-user@your-remote-machine-fqdn-or-ip-here"

Now you can follow the steps above.

For more details see: https://code.visualstudio.com/docs/remote/containers-advanced#_a-basic-remote-example


Building the code
=================

To build our c++ and protobuf generation, we use cmake.  So from inside the docker container (either in the exec shell, or a terminal in VS code) you can run the following commands::


   cd /workspace/tractor
   . setup.bash
   mkdir -p build
   cd build
   cmake -DCMAKE_PREFIX_PATH=/farm_ng/env ..
   make -j$(nproc --ignore=1)
   # run the ipc_logger
   ./build/modules/core/cpp/farm_ng/ipc_logger

To build our frontend and go service try::

   cd /workspace/tractor
   ./env.sh make webservices
   # run the webservices
   PORT=9999 ./env.sh build/go/farm_ng_webservices

Try browsing to : http://localhost:9999/


.. note::

   We'll transition all the build steps to cmake eventually...

Tutorials
=========

.. toctree::
   :maxdepth: 1

   service_tutorial
   program_tutorial
   visualizer_tutorial
