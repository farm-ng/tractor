.. _chapter-program_tutorial:

=================
Writing a Program
=================

This tutorial explains how to write a :ref:`program<section-core_programs>` in C++.

Our program will count down from N to 0, outputting each value to a result file
in the :ref:`blobstore<section-core_blobstore>`.

The full source code for this example is available in `modules/examples <https://github.com/farm-ng/tractor/blob/master/modules/examples>`_.

Prerequisites
=============

This tutorial assumes a working :ref:`C++ development environment<section-development_environment>`.

Define the Interface
====================

Programs, by convention, define a ``Configuration``, ``Status``, and ``Result`` message.

Add a ``countdown.proto`` file with the following contents:

.. literalinclude:: ../modules/examples/protos/farm_ng/examples/countdown.proto
   :language: proto
   :lines: 2-

Add a build target:

.. literalinclude:: ../modules/examples/protos/CMakeLists.txt
   :language: cmake
   :emphasize-lines: 3

Implement the Program
=====================

Add the ``countdown.cpp`` file with the following contents:

.. literalinclude:: ../modules/examples/cpp/farm_ng/countdown.cpp
   :language: cpp
   :lines: 2-

Add a build target:

.. literalinclude:: ../modules/examples/cpp/farm_ng/CMakeLists.txt
   :language: cmake
   :start-after: [docs] countdown
   :end-before: [docs] countdown

Verify that your program runs successfully from the command line, with the expected output.

Add Frontend Support
====================

To make ``programd`` aware of our new program, add the following to
`core/config/programd/programs.json <https://github.com/farm-ng/tractor/blob/master/core/config/programd/programs.json>`_.

.. code-block:: js

  "programs": [
    {
      "id": "countdown",
      "name": "Countdown",
      "description": "Counts down from N to 0, outputting to a file in the blobstore",
      "launchPath": {
        "path": "build/modules/examples/cpp/farm_ng/countdown",
        "rootEnvVar": "FARM_NG_ROOT"
      },
      "launchArgs": ["-interactive"]
    },
    // ...
  ]

.. NOTE ::

  Soon, ``programd`` will accept a search path for configuration files.


Then, follow the instructions in :ref:`chapter-visualizer_tutorial` to add visualizers for the
``CountdownConfiguration``, ``CountdownStatus``, and ``CountdownResult`` messages defined above.

Verify that you can use the :ref:`programs UI<section-frontend_programs>` to start, stop, and monitor your program in the web application.
