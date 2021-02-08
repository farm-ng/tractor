.. _chapter-core_module:

Core
=====

The `core` module provides software infrastructure for all other modules.

Serialization
-------------

farm-ng uses `protocol buffers <https://developers.google.com/protocol-buffers>`_
(v3) for serialization.

A standard approach to serialization allows us to define data structures, library APIs, and network APIs in a format that
is language-neutral, and both forwards and backwards-compatible.

Protobuf's tooling makes it easy to generate serialization code and service stubs for a variety of languages and frameworks.
Currently, we generate code for C++, Python, Go, and Typescript.

.. _section-core_eventbus:

Interprocess Communication
--------------------------

farm-ng uses a lightweight, decentralized, UDP-based **event bus** for interprocess communication.

Processes announce their presence to the bus periodically by sending a serialized ``Announce`` message to a predetermined UDP multicast group.

.. literalinclude:: ../modules/core/protos/farm_ng/core/io.proto
   :language: proto
   :start-after: [docs] announce
   :end-before: [docs] announce

Processes publish ``Events`` to subscribers via UDP unicast.

.. literalinclude:: ../modules/core/protos/farm_ng/core/io.proto
   :language: proto
   :start-after: [docs] event
   :end-before: [docs] event


``Events`` are limited to the size of a single UDP datagram. However, events may include ``Resource`` fields that reference larger chunks of `persistent data`_. Other transports are available for :ref:`image data <Image Data>`.

.. _section-core_blobstore:

Persistent Data
---------------

farm-ng persists data such as configuration files and logs to the filesystem.

The directory containing this data, along with a set of conventions that describe its structure, are referred to collectively as the **blobstore**.

farm-ng provides libraries to facilitate safe, structured interaction with the blobstore.
However, as a directory on disk, the blobstore is also always available for introspection and manipulation via standard command-line or desktop tools.

Whenever possible, we prefer to persist data in standard file formats, rather than as opaque binary blobs.
For example, :ref:`image data <Image Data>` is typically persisted as a ``.png`` or H264-encoded ``.mp4``.
The result is a datastore that's somewhat heterogeneous, but browsable, space-efficient, and compatible with third-party tools.

Logging / Playback
------------------

farm-ng supports logging and replay of event bus traffic via a simple binary log format.

A log consists of binary-serialized ``Event`` messages delimited by a ``uint16`` message length prefix.

.. code-block:: cpp

    void write(const farm_ng::core::Event& event, const std::ofstream& out) {
      std::string packet;
      event.SerializeToString(&packet);
      if (packet.size() > std::numeric_limits<uint16_t>::max()) {
        throw std::invalid_argument("Event is too large");
      }
      uint16_t n_bytes = packet.size();
      out.write(reinterpret_cast<const char*>(&n_bytes), sizeof(n_bytes));
      out << packet;
      out.flush();
    }

It's assumed that a log reader has access to a type registry, or the original message definitions, to properly interpret the contents of a log.

A log replayer is available as a binary and a library.

.. code-block:: bash

  build/modules/core/cpp/farm_ng/log_playback --log foo.log --loop --send --speed 2

.. _section-core_services:

Services
--------
Services are long-lived processes that participate on the event bus.

Services typically encapsulate the core, persistent processes of an application, such as
sensor and actuator drivers, planners, loggers, etc.

Services may be started manually from the command line, but are usually managed via ``docker-compose``, ``systemd`` or a similar service manager.

.. code-block:: yaml

  # docker-compose.yml

  version: "3.3"

  # A minimal set of services, using a mounted volume as the blobstore
  services:
    ipc_logger:
      image: farmng/base:latest
      entrypoint: build/modules/core/cpp/farm_ng/ipc_logger
      environment:
        - BLOBSTORE_ROOT=/blobstore
      volumes:
        - "${BLOBSTORE_ROOT:?err}:/blobstore"
      network_mode: host
    programd:
      image: farmng/base:latest
      entrypoint: python -m farm_ng.core.programd
      environment:
        - BLOBSTORE_ROOT=/blobstore
      volumes:
        - "${BLOBSTORE_ROOT:?err}:/blobstore"
      network_mode: host

.. _section-core_programs:

Programs
--------

Programs are ephemeral processes whose lifecycle can be managed by the rest of the system.

Programs are typically intended for ad hoc use, such as an offline calibration routine.

Programs may be invoked from the command line, or via the event bus, using the ``programd`` service.

Please see :ref:`Writing a Program<chapter-program_tutorial>` for a guide to writing your first program.

Examples
--------

Event (de)serialization
#######################

.. NOTE ::

  Higher-level APIs are also available; these are purely for illustration.

**Python**

.. code-block:: python

  # Serialize
  message = farm_ng.perception.Vec2(x=1.0, y=-1.0)
  event = Event()
  event.name = "odom"
  event.stamp.GetCurrentTime()
  event.data.Pack(message)
  buff = event.SerializeToString()

  # Deserialize
  message = farm_ng.perception.Vec2()
  event.data.Unpack(message)

**Typescript**

.. code-block:: typescript

  // Serialize
  const event = Event.fromPartial({
    name: "odom",
    stamp: new Date(),
    data: {
      typeUrl: "type.googleapis.com/farm_ng.perception.Vec2",
      value: Vec2.encode(Vec2.fromPartial({x: 1.0, y: -1.0)).finish(),
    },
  });
  const buff = Event.encode(event).finish()

  // Deserialize
  const { name, stamp, data } = event;
  const value = Vec2.decode(data.value);

Single process logging
######################

**C++**

.. code-block:: cpp

  namespace farm_ng {
  namespace core {

  // Request a new resource to write to in the blobstore
  std::pair<Resource, boost::filesystem::path> resource_path =
    GetUniqueArchiveResource("events", "log", "application/farm_ng.eventlog.v1");

  // Write a message to the event log
  farm_ng::perception::Vec2 odom;
  odom.set_x(1.0);
  odom.set_y(-1.0);
  EventLogWriter log_writer(resource_path.second);
  log_writer.Write(MakeEvent("odom", odom));

  } // namespace core
  } // namespace farm_ng

Multi-process logging
######################

**C++**

.. code-block:: cpp

  namespace farm_ng {
  namespace core {

  // Preconditions:
  // - An ipc-logger process is also running on `bus`
  void LogOdom(const EventBus& bus, const farm_ng::perception::Vec2& odom) {
    LoggingStatus log = StartLogging(bus_, configuration_.name());
    bus.Send(MakeEvent(bus.GetName() + "/odom", odom));
  }

  } // namespace core
  } // namespace farm_ng
