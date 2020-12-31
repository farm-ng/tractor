.. _chapter-program_tutorial:

=================
Writing a Program
=================

This tutorial explains how to write a Hello World :ref:`program<section-programs>` in C++.

Introduction
============

Prerequisites
=============

Define the Interface
====================

Implement the Program
=====================

Add to programd
===============

To make ``programd`` aware of our new program, add to
`core/config/programd/programs.json <https://github.com/farm-ng/tractor/blob/master/core/config/programd/programs.json>`_.

.. code-block:: json

  "programs": [
    {
      "id": "TODO",
      "name": "TODO",
      "description": "TODO",
      "launchPath": {
        "path": "TODO",
        "rootEnvVar": "FARM_NG_ROOT"
      },
      "launchArgs": ["-interactive"]
    },
    // ...
  ]

.. NOTE ::

  Soon, ``programd`` will accept a search path for configuration files.


Add visualization
=================

The final step of implementing a program is adding visualization.
Follow the instructions in :ref:`chapter-visualizer_tutorial` to add visualizers for the
configuration, status, and result messages defined above.
