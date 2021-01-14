.. _chapter-getting_started:

===============
Getting Started
===============

This guide describes how to install and extend farm-ng software.

If you're already familiar with the farm-ng codebase, and looking for packaged binaries, please see
:ref:`Docker Releases<getting_started-docker-releases>`.

.. _section-development_environment:

Setting up your Development Environment
=======================================
We have chosen to encapsulate the development environment in a Docker container, to simplify reproducibility.
It's also possible to build on bare metal, and the Dockerfiles serve as documentation for our system dependencies.

We fully support Visual Studio Code, and describe a recommended configuration below.

This workflow has been tested on development machines running Ubuntu (18.04, 20.04) Desktop, NVidia's Jetson Nano and Xavier platforms, and Mac OS.
Windows support should also be possible, but hasn't been tested.

To get started, you'll need a few system tools installed:

- ``git``
- ``docker``
- ``docker-compose``
- ``vscode``

Install ``git``:

.. tabs::

   .. group-tab:: Ubuntu

      .. code-block:: bash

        sudo apt install git


   .. group-tab:: Mac OSX

      .. code-block:: bash

         brew install git

   .. group-tab:: Windows

      TODO

Install ``docker``:

.. tabs::

   .. group-tab:: Ubuntu

      Please follow the `official Docker installation <https://docs.docker.com/engine/install/ubuntu/>`_ instructions.

      Ensure that you have added your user to the ``docker`` group, and logged out and back in:

      .. code-block:: bash

        sudo usermod -aG docker $USER


   .. group-tab:: Mac OSX

      Please follow the `official Docker installation <https://docs.docker.com/docker-for-mac/install/>`_ instructions.

   .. group-tab:: Windows

      TODO


Install ``docker-compose``:

.. tabs::

   .. group-tab:: Ubuntu

      Using pip:

      .. code-block:: bash

        sudo apt install pip3
        sudo pip3 install docker-compose


   .. group-tab:: Mac OSX

      ``docker-compose`` is included in the `Docker Desktop for Mac` installer used to install ``docker``.

   .. group-tab:: Windows

      TODO

Install ``vscode``:

.. tabs::

   .. group-tab:: Ubuntu

      Please follow the `official Visual Studio Code installation <https://code.visualstudio.com/Download/>`_ instructions.

   .. group-tab:: Mac OSX

      Please follow the `official Visual Studio Code installation <https://code.visualstudio.com/Download/>`_ instructions.

   .. group-tab:: Windows

      TODO


Get the code
------------

.. code-block:: bash

   git clone https://github.com/farm-ng/tractor.git

Start the development container
-------------------------------

``docker-compose`` can be used to start the development Docker container. It mounts the repository from your host as a volume.

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


To stop the container, press ``Ctrl-C``.

Now the Docker container is up and running.  You can ``exec`` into it, interactively, and open as many shells as you like with the following command:

.. code-block::

   docker exec -it docker_devel_1 bash


Try the following, on your host machine, to configure the ``cmake`` build inside the container:

.. code-block:: bash

   docker exec docker_devel_1 bash -c ". setup.bash && mkdir -p build && cd build && cmake -DCMAKE_PREFIX_PATH=/farm_ng/env -DBUILD_DOCS=True .."


You should see something similar to::

   -- The C compiler identification is GNU 7.5.0
   ...
   -- Configuring done
   -- Generating done
   -- Build files have been written to: /workspace/tractor/build


Now that the code is configured, you should see a ``build/`` directory on the host machine with a ``Makefile`` and ``CMakeCache.txt``

Let's try building these docs::

   docker exec docker_devel_1 /workspace/tractor/env.sh make -C build docs


And run a server to host them::

   docker exec docker_devel_1 bash -c "cd build/doc/sphinx && python3 -m http.server 8000"

Try browsing to http://localhost:8000/getting_started.html#start-the-development-container

Now take a moment to study the contents of ``docker/docker-compose.devel.yaml``:

.. literalinclude:: ../docker/docker-compose.devel.yml
   :language: yaml

- The source directory is mounted at ``/workspace/tractor``.
- A named volume is mounted at ``/root`` -- this allows the container to persist bash history and other cached variables.
- A host directory is mounted as the ``BLOBSTORE_ROOT`` for :ref:`persistent storage of application data<section-core_blobstore>`.


.. note::

   Currently the Docker container user is ``root``.  We want to fix that shortly, so it matches your ``$UID`` on the host...
   living on the edge here!



Visual Studio Code setup
------------------------

Now that you're comfortable execing into the development container let's boot up ``vscode``.

Start ``vscode`` in the normal way and install the extension ``ms-vscode-remote.remote-containers``.

Attach to the container ``docker_devel_1`` using **Remote-Containers: Attach to Running Container**.

.. note::

   For additional background, see vscode documentation on `container-based workflows <https://code.visualstudio.com/docs/remote/attach-container>`_.

Once you're attached, open the workspace file ``/workspace/tractor/workspace.code-workspace`` and install the suggested plugins.

Wait while ``vscode`` installs the extensions.

Try building with ``cmake`` in ``vscode`` by pressing ``F7``.

- Click `Unspecified` when prompted to `Select a Kit`.
- You can see the build output in the `CMake/Build` output window.

Developing on a Remote Host
------------------------------

This workflow above should also work if you're connected to a remote host.
This is how we develop on remote machines such as robots.

To attach to a remote container over SSH just add the following to your `User Settings <https://code.visualstudio.com/docs/getstarted/settings#_settings-file-locations>`_:

.. code-block::

   "docker.host":"ssh://your-remote-user@your-remote-machine-fqdn-or-ip-here"

Restart ``vscode``, and the workflow above should connect you to the Docker container on your remote host.

.. note::

   For additional background, see vscode documentation on `remote container workflows <https://code.visualstudio.com/docs/remote/containers-advanced#_a-basic-remote-example>`_.


Building the code
-----------------

To build our C++ and protobuf generated code, we use ``cmake``.  From inside the Docker container (either in the ``exec`` shell, or a terminal in VS code) you can run the following commands:

.. code-block:: bash

   cd /workspace/tractor
   . setup.bash
   mkdir -p build
   cd build
   cmake -DCMAKE_PREFIX_PATH=/farm_ng/env ..
   make -j$(nproc --ignore=1)

   # Run the ipc_logger
   cd /workspace/tractor
   ./build/modules/core/cpp/farm_ng/ipc_logger

To build our webservices (Typescript and Go) run:

.. code-block:: bash

   cd /workspace/tractor
   ./env.sh make webservices

   # Run the webservices
   PORT=9999 ./env.sh build/go/farm_ng_webservices

Try browsing to `<http://localhost:9999/>`_.


.. note::

   We'll transition all the build steps to ``cmake`` eventually...

Tutorials
=========

With a working development environment, you're ready to explore our tutorials:

.. toctree::
   :maxdepth: 1

   service_tutorial
   program_tutorial
   visualizer_tutorial

.. _getting_started-docker-releases:

Docker Releases
===============

The following images are available on Dockerhub:

- `farmng/base <https://hub.docker.com/repository/docker/farmng/base>`_: All C++, Go, web server and web applications, and Python binaries and all source code.

- `farmng/devel <https://hub.docker.com/repository/docker/farmng/devel>`_: All source code and a base development environment with dependencies pre-installed.

Docker tags are used as follows:

- ``edge``: Built by CI for the latest commit on ``master``.

- ``latest``: Built by CI for the latest `Github release <https://github.com/farm-ng/tractor/releases>`_

- ``<x.y.z>``: Built by CI for the `Github release <https://github.com/farm-ng/tractor/releases>`_ ``x.y.z``

- ``<sha>``: Built ad hoc for a specific git commit.
