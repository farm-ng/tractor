.. _chapter-visualizer_tutorial:

====================
Writing a Visualizer
====================

This tutorial explains how to write a visualizer for a custom data type,
or a custom visualizer for an existing data type, in Typescript.

Introduction
============

A ``Visualizer`` is defined by the following interface:

.. code-block:: typescript

  export interface Visualizer<T extends EventType = EventType> {
    id: VisualizerId;
    Component: React.FC<VisualizerProps<T>>;
    options: VisualizerOptionConfig[];
    types: EventTypeId[] | "*";
    Element?: React.FC<SingleElementVisualizerProps<T>>;
    Form?: React.FC<FormProps<T>>;
    Marker3D?: React.FC<SingleElementVisualizerProps<T>>;
  }

TODO: Comments

A visualizer may choose to implement any subset of the visualization interfaces (2D, 3D, form, etc.)
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

Define your visualizer
======================

In a new file, define your new visualizer.

.. code-block:: typescript

  // CustomVec2Visualizer.ts

  export const CustomVec2Visualizer = {
    id: "CustomVec2",
    types: ["type.googleapis.com/farm_ng.perception.Vec2"],
    options: [],
  };

Add this visualizer to the visualization registry.

.. code-block:: typescript

  // registry/visualization.ts

  export const visualizerRegistry: { [k: string]: Visualizer } = [
    CustomVec2Visualizer,
    // ...

.. _section-visualization_priority:

**Visualization Priority**

The order of visualizers in the registry determines their priority. In other words, if you'd like the default visualization
for ``Vec2`` to be your custom visualizer, put it before any existing ``Vec2`` visualizers.

The most general visualizers (``JSONVisualizer``, ``TimeSkewVisualizer``) remain at the bottom.

Implement visualization
=======================

Comments below explain how to implement 2D, 3D, and Form visualization.

.. code-block:: typescript

  // CustomVec2Visualizer.ts

  // TODO: Full class

  const CustomVec2Element: React.FC<SingleElementVisualizerProps<
    Vec2
  >> = (props) => {
    const {
      value: [timestamp, value],
    } = props;

    return (
      <Card timestamp={timestamp} json={value}>
        <KeyValueTable
          records={[
            ["x", value.x],
            ["y", value.y],
          ]}
        />
      </Card>
    );
  };

  export const CustomVec2Visualizer = {
    id: "CustomVec2",
    types: ["type.googleapis.com/farm_ng.perception.Vec2"],
    options: [],
    Element: CustomVec2Element,
  };

TODO: .tsx syntax highlighting

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
