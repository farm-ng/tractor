.. _chapter-infrastructure:

Infrastructure
==============

Farm-ng provides a set of common infrastructure for the development of autonomous applications.

Serialization
-------------

Farm-ng uses `Protocol Buffers (v3) <https://developers.google.com/protocol-buffers>`_
for serialization.

A standard approach to serialization allows us to define data structures, library APIs, and service APIs in a format that
is language-neutral, and both forwards and backwards-compatible as schemas evolve.

For more about the the choice of protocol buffers, see TODO.

Interprocess Communication
--------------------------

Farm-ng uses a lightweight, decentralized UDP-based protocol called the **Event Bus** for interprocess communication.

UDP packets contain a single binary-serialized protobuf that is either an ``Event`` or an ``Announce``.

Processes announce their presence to the bus periodically via UDP multicast, and publish events to their peers via UDP unicast.

The event bus supports the concept of `subscriptions` to limit the traffic received by a peer.

While UDP datagrams are limited to 65535 bytes, messages may include references (``Resources``) to larger chunks of persistent data (see below).

Persistent Data
---------------

Farm-ng persists data such as configuration and logs to the filesystem.

The directory containing this data, along with a set of conventions that describe its structure, are referred to collectively as the **Blobstore**.

Farm-ng provides libraries to facilitate safe, structured interaction with the blobstore.
However, as a directory on disk, the blobstore is also always available for introspection and manipulation via standard command-line or desktop tools.

Whenever possible, we prefer to persist data in standard file formats, rather than as opaque binary blobs.
For example, image data is persisted as a ``.png`` or H264-encoded ``.mp4``.
The result is a datastore that's somewhat heterogeneous, but highly browsable, space-efficient, and compatible with third-party tools.

Logging / Playback
------------------

Farm-ng supports logging and replaying eventbus traffic via a simple binary log format.

A log consists of binary-serialized ``Event`` messages delimited by a ``uint32`` message length prefix.

``Event`` messages encode events as a ``google.protobuf.Any``, so it's assumed that a log reader
has access to a type registry, or the original message definitions, to properly interpret the contents of a log.

A log replayer is available as a binary and a library in the ``core`` module.

Services and Programs
---------------------
Services and programs are software processes that participate on the eventbus.

Services typically encapsulate the core, persistent processes of an application, such as
sensor and actuator drivers, planners, loggers, etc.

Services may be started manually from the command line, but are usually managed via ``systemd`` or a similar service manager.

Programs typically encapsulate ephemeral processes intended for ad hoc use, such as a calibration routine.

Programs may be invoked from the command line, or via the eventbus, using the ``programd`` service.

Frontend
--------

Farm-ng provides a browser-based frontend as a GUI and graphical debugging tool.

The frontend participates on the eventbus via a WebRTC proxy, allowing it to introspect eventbus traffic and interact
with running services and programs.

It also exposes CRUD functionality for the blobstore, and a GUI for ``programd``.

The frontend aims to support an extensible approach to 2D and 3D visualization, such that
it is easy to build custom visualizations for application-specific data types, and
build full application-specific web apps.

Hardware
--------

Farm-ng aims to support a wide array of sensors, actuators, and compute platforms.

Supported hardware to date includes camera drivers (Intel Realsense, Azure Kinect), motor drivers (ODrive, Vesc),
and NVIDIA Jetson compute.

Packaging and Deployment
------------------------

Farm-ng uses a concept of `modules` to organize its source code and
package source and binaries for downstream consumption.

Modules encapsulate a logical area of functionality, such as `perception` or `kinematics`, and
typically include source code in multiple languages, protobuf definitions, and integration tests.

Modules may depend on other modules. Currently they may be consumed by downstream projects
via source tarballs; we plan to support package management tools such as `debian`, `pip`, or `conda` in the future.

Farm-ng uses `docker-compose` for software deployment, and provides reference Dockerfiles and images on Dockerhub.
