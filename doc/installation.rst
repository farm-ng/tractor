.. _chapter-installation:

============
Installation
============

Farm-ng libraries and applications may be installed with Docker, or built from source.

Currently, building from source is only supported on Ubuntu 18.04.

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

Building from source
====================
.. _section-source:

- Use a release
- Clone the git repository

.. code-block:: bash

       git clone https://github.com/farm-ng/tractor.git

.. _section-dependencies:

Dependencies
------------

farm-ng requires a C++17-compliant compiler.

farm-ng relies on a number of open source libraries:

- `CMake <http://www.cmake.org>`_ 3.5 or later **required**.

- `glog <https://github.com/google/glog>`_ 0.3.1 or
  later. **Recommended**

  TODO notes on this dependency


- `gflags <https://github.com/gflags/gflags>`_. Needed to build
  examples and tests and usually a dependency for glog.

.. _section-linux:

Ubuntu 18.04
============

Start by installing all the dependencies.

.. code-block:: bash

     # CMake
     sudo apt-get install cmake

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
