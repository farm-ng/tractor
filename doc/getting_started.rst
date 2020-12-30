.. _chapter-getting_started:

============
Getting Started
============

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

Building from source
====================
The following instructions describe building the entire farm-ng source tree.
The same principles apply to building individual modules, which are made available as tarballs on each
`Github release <https://github.com/farm-ng/tractor/releases>`_.

a) Download a release **or**

.. code-block:: bash

       wget https://github.com/farm-ng/tractor/archive/v0.7.0.tar.gz


b) Clone the farm-ng repository

.. code-block:: bash

       git clone https://github.com/farm-ng/tractor.git

C++
---

farm-ng's C++ codebase requires a C++17-compliant compiler and depends on a
number of open source libraries:

- `CMake <http://www.cmake.org>`_ 3.?? or later **required**.

- `glog <https://github.com/google/glog>`_ 0.3.?? or
  later. **required**

- TODO

Step-by-step
------------

Start by installing all the dependencies.

.. code-block:: bash

     # TODO
     sudo apt-get install <TODO>

We are now ready to build, test, and install farm-ng.

.. code-block:: bash

 tar zxf farm-ng-0.0.1.tar.gz
 mkdir farm-ng-bin
 cd farm-ng-bin
 cmake ../farm-ng-0.0.1
 make -j3
 make test
 make install

Now try running one of the included applications TODO

.. code-block:: bash

 modules/core/cpp/farm_ng/ipc_logger

This runs ...

.. code-block:: bash

  TODO

Tutorials
=========

.. toctree::
   :maxdepth: 1

   helloworld_tutorial
