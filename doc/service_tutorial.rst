.. _chapter-service_tutorial:

=================
Writing a Service
=================

This tutorial explains how to write a Hello World :ref:`service<section-core_services>` in C++.

Our service will publish ``helloworld/status`` events, subscribe to ``helloworld/command`` events,
and interact with the ``ipc_logger`` service.

The full source code for this example is available in `modules/examples <https://github.com/farm-ng/tractor/blob/master/modules/examples>`_.

Prerequisites
=============

This tutorial assumes a working :ref:`C++ development environment<section-development_environment>`.

Define data models
==================

First, define a ``Status`` and ``Command`` message.

Add a ``helloworld.proto`` file with the following contents:

.. literalinclude:: ../modules/examples/protos/farm_ng/examples/helloworld.proto
   :language: proto
   :lines: 2-

Add a build target:

.. literalinclude:: ../modules/examples/protos/CMakeLists.txt
   :language: cmake
   :emphasize-lines: 4

Implement the Service
=====================

Add the ``helloworld.cpp`` file with the following contents:

.. literalinclude:: ../modules/examples/cpp/farm_ng/helloworld.cpp
   :language: cpp
   :lines: 2-

Add a build target:

.. literalinclude:: ../modules/examples/cpp/farm_ng/CMakeLists.txt
   :language: cmake
   :start-after: [docs] helloworld
   :end-before: [docs] helloworld

Verify that your service runs successfully from the command line, with the expected output.
