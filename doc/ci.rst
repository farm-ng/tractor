Continuous Integration
======================

For continuous integration we use github workflow actions to:

- run linters and static analyzers
- publish docker development environment images
- build and run tests on pull requests
- publish release artifacts and docker images
- run hardware-in-the-loop regression tests
- run data-driven regression tests

You can find our workflows under `//.github/workflows <https://github.com/farm-ng/tractor/blob/master/.github/workflows>`_.

Our CI system is iteratively improving, and some of the above is still a work-in-progress.

For hardware-in-the-loop and data-driven regression tests we use a private repository and self-hosted runners.

Building farmng/devel
---------------------


The ``farmng/devel`` docker image contains all of the build dependencies for our repository.
It's a big image and some of the build steps take time.
Luckily it changes slowly over time.

Pushing to the branch ``devel`` will cause CI to build ``farmng/devel`` using the github workflow located at
`//.github/workflows/devel.yml <https://github.com/farm-ng/tractor/blob/devel/.github/workflows/devel.yml>`_.


third party dependencies
++++++++++++++++++++++++

We prefer to depend on Ubuntu LTS packaged dependencies brought in via apt,
or the upstream pip, go, and npm package managers.
However some libraries are either too new to have packages (e.g. grpc), too esoteric (e.g. Sophus), or require
different build flags (e.g. opencv with gstreamer support).
These can be complex dependencies to build from source, and take a *long time* on devices like the Jetson Nano,
so we use docker multi-stage builds to manage these vendored dependencies.
We could have chosen to build debians, conda packages, or snaps, but since we're already in docker land...

First, the ``devel.yml`` github action builds a docker image for each of the third party dependencies we build from source.

.. literalinclude:: ../.github/workflows/devel.yml
   :language: yaml
   :end-before: [docs] third_party

The resulting images only contain one layer, with headers and binaries
compiled and installed under the `prefix FHS <https://en.wikipedia.org/wiki/Filesystem_Hierarchy_Standard>`_
``/farm_ng/env``.
The files in this layer are meant to be copied into other containers,
and the images themselves are small compared to the build-time requirements.

The images are pushed to the dockerhub repository ``farmng/build-<third party name>``. For example, ``farmng/build-grpc`` is built from
`//docker/devel/grpc.Dockerfile <https://github.com/farm-ng/tractor/blob/master/docker/devel/grpc.Dockerfile>`_.

.. literalinclude:: ../docker/devel/grpc.Dockerfile
   :language: dockerfile

Next, this workflow builds our devel image (`//docker/devel/devel.Dockerfile <https://github.com/farm-ng/tractor/blob/master/docker/devel/devel.Dockerfile>`_),
which depends on the third party images described above:

.. literalinclude:: ../docker/devel/devel.Dockerfile
   :language: dockerfile
   :start-after: [docs] copy_third_party
   :end-before: [docs] copy_third_party

.. note::

    We support x86 and the jetson platform, and would like to extend this to build for platforms in the near future using:

    - https://github.com/docker/setup-buildx-action
    - https://github.com/marketplace/actions/build-and-push-docker-images#multi-platform-image
