.. _chapter-installation:

============
Installation
============

Getting the source code
=======================
.. _section-source:

- Use a release
- Clone the git repository

.. code-block:: bash

       git clone https://github.com/farm-ng/tractor.git

.. _section-dependencies:

Dependencies
============

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
