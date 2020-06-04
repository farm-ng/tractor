## Serialization

To facilitate type safe data persistence, and cross language API
definitions, we've decided to use protobuf for message types and API
service definitions.

We considered several options (json, flatbuffer, capnproto, protobuf),
protobuf appeared to be the most stable, well supported choice, with
the ability to define custom API generators with its service syntax.
In particular we were looking for something with type-script, python,
go, and C++ support.


## Data logging
For testing and development, it is necessary to save logs that can be
used to reconstruct the state of the tractor offline.  Since we are
using protobuf for message serialization, it seems natural to have a
simple file stream container for protobuf messages that can be used
for such logging.

Design discussion: https://github.com/farm-ng/tractor/issues/24

We're trying the proio format and corresponding proio-python library
for datalogging as it supports binary protobufs, is well documented,
and well implemented in go, python, c++, with some command line tools.
It should easily be extensible for use in a logging/playback use case.
