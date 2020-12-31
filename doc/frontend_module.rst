.. _chapter-frontend_module:

Frontend
========

The *frontend* module provides webserver and frontend code for building GUIs and graphical debugging tools.

Client/Server Model
-------------------

The frontend is a single-page React web application, implemented in Typescript, served by a Go backend.

**WebRTC**

A ``POST`` request to ``/twirp/farm_ng.frontend.WebRTCProxyService/InitiatePeerConnection``
initiates a WebRTC connection with the server.

.. NOTE ::

  In a common WebRTC use case like 1:1 video conferencing, a signaling server brokers a connection
  between two peers on the internet. Here, the signaling server and the remote peer are one and the same --
  all communication is on the local area network.

The negotiated WebRTC connection includes both a bidirectional data stream (for :ref:`event bus<section-core_eventbus>` data), and a
server-to-client media stream (for streaming video).

The server acts as a proxy between the frontend and the robotics application, forwarding:

- browser ``Events`` from the WebRTC data channel to the event bus
- event bus ``Events`` to the WebRTC data channel to the browser
- RTP packets to the WebRTC video channel to the browser

The environment variable ``RTP_PORT`` specifies a port for the server to listen for RTP packets from the robotics application. ``RTP_PORT`` defaults to ``5000``.

**Blobstore CRUD**

A ``GET`` request to ``/blobstore/<path>`` returns:

- the contents of the file at ``<path>``, if ``<path>`` is a valid path to a file in the :ref:`blobstore<section-core_blobstore>`
- a ``File`` message describing the contents of ``<path>``, if ``<path>`` is a valid path to a directory in the blobstore
- else a 404 status code

.. literalinclude:: ../modules/core/protos/farm_ng/core/resource.proto
   :language: proto
   :start-after: [docs] file
   :end-before: [docs] file

A ``POST`` request to ``/blobstore/<path>``:

- returns a 500 status code if ``<path>`` is a directory in the blobstore or an invalid path
- else writes the payload of the request to ``<path>`` (overwriting existing content)

The environment variable ``BLOBSTORE_ROOT`` specifies the blobstore root.

**Static Files**

A ``GET`` request to ``/<path>`` will return

- the contents of the file at ``<path>``, if ``<path>`` is a valid path to a file in the server's static directory
- else ``index.html``, to bootstrap the single-page app

.. _section-frontend_blobstore:

Blobstore UI
------------
.. image:: https://via.placeholder.com/1920x1080.png?text=Blobstore+Screenshot

The frontend provides a UI for browsing and updating the :ref:`blobstore<section-core_blobstore>`.

- Known filetypes (currently determined by a heuristic based on the path) with a registered visualizer are visualized.
- Known filetypes with a registered form visualization may be edited and saved.
- All other filetypes are downloaded, or opened in a new tab in the browser.

.. _section-frontend_programs:

Programs UI
-----------
.. image:: https://via.placeholder.com/1920x1080.png?text=Programs+Screenshot

The frontend provides a GUI for interacting with ``programd`` to start, stop, and monitor :ref:`programs<section-core_programs>`.

The program listing is populated by ``ProgramSupervisorStatus`` events, published periodically by ``programd``.

.. literalinclude:: ../modules/core/protos/farm_ng/core/programd.proto
   :language: proto
   :start-after: [docs] programd_status
   :end-before: [docs] programd_status

Events published with the name ``<program_name>/status``, where ``<program_name>`` corresponds to the running program, will be visualized.

.. _section-frontend_scope:

Scope UI
--------
.. image:: https://via.placeholder.com/1920x1080.png?text=Scope+Screenshot

The scope is a visual debugger for :ref:`event bus<section-core_eventbus>` events.

When the scope is *streaming* (via the *Stream* button), event bus events are stored in a client-side, in-memory FIFO queue.
The maximum age of data in the queue is determined by the *Expiration* option.
Existing data is cleared when *Stream* is clicked, or on page reload.

In a *panel* we can visualize data in the queue, selected by time, type, and name (in that order).

  - Use the timeline control and *Throttle* menu to select a time range / resolution
  - Use the *Data Type* menu to select a data type
  - Use the *Tag Filter* menu to select a subset of event names

For each type of data, you may choose from one of multiple visualizers [`registry/visualization.ts <https://github.com/farm-ng/tractor/blob/master/modules/frontend/frontend/src/registry/visualization.ts>`_].
Each visualizer may expose multiple visualization options.

The `+` button adds additional panels.

Extensibility
-------------
The frontend aims to be extensible, such that it is easy to build custom 2D/3D visualizations
for application-specific data types, and even fully custom web apps.

As designed, the architecture optimizes for extensibility of visualization *in the frontend*, rather than in the robotics application.
There are tradeoffs, but we believe that this approach empowers developers to build visualizations (and applications) with
the DOM, Javascript, React, HTTP, and all the features of the browser platform, as opposed to an approach
that provides a generic visualization API for C++ and Python developers.

You may wish to extend the frontend by addding visualizers for your custom data types,
or adding alternative visualizers for existing data types. This process is explained in the :ref:`Writing a Visualizer<chapter-visualizer_tutorial>` tutorial.

Alternatively, you may wish to build an entirely new React web application, using the frontend module as a JS/TS library from which to
pull React components, React hooks, utilities, and so on.


Examples
--------

.. _section-frontend_development_server:

**Run a development server**

.. literalinclude:: ../modules/frontend/frontend/README.md
   :language: bash
   :start-at: # To launch a development server
   :end-at: # Saved changes will automatically rebuild

.. _section-frontend_new_data_type:

**Add support for a new data type**

.. code-block:: ts

  // Ensure the relevant ts-proto generated code is imported in package.json
  "dependencies": {
    "@farm-ng/genproto-calibration": "link:../../../build/modules/calibration/protos/ts",
    // ...
  }

  // Add the type URL and the ts-proto generated message class to the event registry
  export const eventRegistry = inferKeys({
    "type.googleapis.com/farm_ng.calibration.Foo": Foo,
    // ...
  });

  // Add the ts-proto generated message type to the union type EventType
  export type EventType =
  | Foo
  // ...

**Add support for a new program**

TODO
