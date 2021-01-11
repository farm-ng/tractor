.. _chapter-visualizer_tutorial:

====================
Writing a Visualizer
====================

This tutorial explains how to write a visualizer for a custom data type,
or a custom visualizer for an existing data type.

Visualizers are implemented as React components, in Typescript.

The full source code for this example is available in `modules/frontend <https://github.com/farm-ng/tractor/blob/master/modules/frontend>`_.

Introduction
============

A ``Visualizer`` is defined by the following interface:

.. literalinclude:: ../modules/frontend/frontend/src/registry/visualization.ts
   :language: typescript
   :start-after: [docs] Visualizer
   :end-before: [docs] Visualizer

A visualizer may choose to implement any subset of the visualizations (single-element, multi-element, 3D, form).
In this tutorial we'll implement all of them.

Prerequisites
=============
1. Run a frontend development server

  Follow the instructions for :ref:`launching a development server<section-frontend_development_server>`.
  Ensure your development environment is properly compiling on save, highlighting compilation errors, etc.

2. Ensure the data type you'd like to visualize is registered

  Follow the instructions for :ref:`registering a new data type<section-frontend_new_data_type>`,
  or work with an existing data type. In this tutorial we'll work with the existing data type
  ``farm_ng.perception.Vec2``.

Implement visualizer
=======================

Add a ``CustomVec2Visualizer.tsx`` file with the following contents:

.. literalinclude:: ../modules/frontend/frontend/src/components/scope/visualizers/CustomVec2Visualizer.tsx
   :language: typescript

Add to the visualizer registry
==============================

Add the following to ``registry/visualization.ts``:

.. code-block:: typescript

  // registry/visualization.ts

  // import { CustomVec2Visualizer } from ...

  export const visualizerRegistry: { [k: string]: Visualizer } = [
    CustomVec2Visualizer,
    // ...

.. _section-visualization_priority:

**Visualization Priority**

The order of visualizers in the registry determines their priority. If you'd like the default visualization
for ``Vec2`` to be your custom visualizer, put it before any existing ``Vec2`` visualizers.

The most general visualizers (``JSONVisualizer``, ``TimeSkewVisualizer``) remain at the bottom.

Verify in the web application
=============================

Publish ``Vec2`` messages on the event bus and verify that you can visualize them
with your new visualizer in the :ref:`scope<section-frontend_scope>`.

If you have ``Vec2`` messages stored in the blobstore, verify that you can
visualize and edit them with your new visualizer in the :ref:`blobstore UI<section-frontend_blobstore>`.

.. NOTE ::

  You may need to extend the blobstore browser's ``bestGuessEventType`` function to provide
  a hint about the data type of new file paths in the blobstore.

If you have a program that emits status messages that include ``Vec2`` messages, verify that you can visualize
them with your new visualizer in the :ref:`programs UI<section-frontend_programs>`. If your program supports
configuration that includes a ``Vec2``, verify that you can edit it.

.. NOTE ::

  The blobstore UI and programs UI use a data type's :ref:`default visualization<section-visualization_priority>`.

Utilities and Hooks
===================

Explore the *frontend* module for re-usable React components (e.g. ``KeyValueTable``),
React hooks (e.g. ``useFetchResource``), and utilities (e.g. ``colorGenerator``) that may be useful
in implementing your custom visualization.
