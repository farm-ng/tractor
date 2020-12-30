.. _chapter-core_module:

Core
=====

The `core` module provides infrastructure for all other modules.

Serialization
-------------

farm-ng uses `protocol buffers <https://developers.google.com/protocol-buffers>`_
(v3) for serialization.

A standard approach to serialization allows us to define data structures, library APIs, and network APIs in a format that
is language-neutral, and both forwards and backwards-compatible.

Protobuf's tooling makes it easy to generate serialization code and service stubs for a variety of languages and frameworks.
Currently, we generate code for C++, Python, Go, and Typescript.


Interprocess Communication
--------------------------

farm-ng uses a lightweight, decentralized UDP-based **Event Bus** for interprocess communication.

Processes announce their presence to the bus periodically by sending a serialized ``Announce`` message to a predetermined UDP multicast group.

.. code-block:: proto

  message Announce {
    string host = 1;
    int32 port = 2;
    string service = 3;
    google.protobuf.Timestamp stamp = 4;
    google.protobuf.Timestamp recv_stamp = 5;
    repeated Subscription subscriptions = 6;
  }

  message Subscription {
    // A regular expression that acts as a filter on event names.
    string name = 1;
  }

Processes publish ``Events`` to subscribers via UDP unicast.

.. code-block:: proto

  message Event {
    google.protobuf.Timestamp stamp = 1;
    string name = 2;
    google.protobuf.Any data = 3;
    google.protobuf.Timestamp recv_stamp = 4;
  }

``Events`` are limited to the size of a single UDP datagram. However, events may include ``Resource`` fields that reference larger chunks of `persistent data`_. Other transports are available for :ref:`image data <Image Data>`.

Persistent Data
---------------

farm-ng persists data such as configuration files and logs to the filesystem.

The directory containing this data, along with a set of conventions that describe its structure, are referred to collectively as the **Blobstore**.

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

Services
--------
Services are long-lived processes that participate on the event bus.

Services typically encapsulate the core, persistent processes of an application, such as
sensor and actuator drivers, planners, loggers, etc.

Services may be started manually from the command line, but are usually managed via ``docker-compose``, ``systemd`` or a similar service manager.

TODO: docker-compose example

Programs
--------

Programs are ephemeral processes whose lifecycle can be managed by the rest of the system.

Programs are typically intended for ad hoc use, such as an offline calibration routine.

Programs may be invoked from the command line, or via the event bus, using the ``programd`` service.

Programs adhere to a set of conventions.

TODO: Documenting communication pattern for programd

Examples
--------

Event (de)serialization
#######################
(in each language)


Single process logging
######################

Multi-process logging
######################
