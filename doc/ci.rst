Continuous Integration
======================

For continuous integration we use github workflow actions to:
- check that pre-commit lint checks and static analysis is valid
- build our docker development environment images
- building and running tests on pull requests
- publishing release artifacts and docker images
- hardware in the loop regression testing
- data driven regression testiong.

You can find our workflows under `//.github/workflows <https://github.com/farm-ng/tractor/blob/master/.github/workflows>`_.

Our CI system is iteratively improving, and some of the above is still a work and progress.

For hardware in the loop and data driven regression tests we use a private repository and self-hosted runners.

Building farmng/devel
---------------------


``farmng/devel`` contains all of the build dependencies for our repository. Its a big image and some of the build steps take time.
Luckily it changes slowly over time.

Pushing to the branch ``devel`` will cause CI to build ``farmng/devel`` using the github workflow located
`//.github/workflows/devel.yml <https://github.com/farm-ng/tractor/blob/devel/.github/workflows/devel.yml>`_.


third party dependencies
++++++++++++++++++++++++

We mostly depend on Ubuntu LTS packaged dependencies brought in via apt,
or the upstream pip, go, npm, and yarn package managers, to keep our build simple.
However some libraries are either too new such as grpc to have packages, esoteric like Sophus, or we need to
different build flags such as building opencv with gstreamer support.
These can be complex dependencies to build from source, and take a *long time* on devices like the jetson nano,
so we use docker multi-stage builds to manage these vendored in dependencies.
We could have chosen to build debians, conda packages, or snaps, but since we're already in docker land...

First the devel github action builds a docker image for each our third party dependenencies we build from source.

.. literalinclude:: ../.github/workflows/devel.yml
   :language: yaml
   :end-before: doc_third_party

The resulting images only contain one layer, with the compiled installed headers/binaries,
located under the `prefix FHS <https://en.wikipedia.org/wiki/Filesystem_Hierarchy_Standard>`_
``/farm_ng/env``.
The files in this layer are meant to be copied into other containers,
and the images themselves are small compared to the build time requirements.

The images are pushed to the dockerhub registry under ``farmng/build-<third party name>``, for example: ``farmng/build-grpc`` is built from the Dockerfile
`//docker/grpc.Dockerfile <https://github.com/farm-ng/tractor/blob/devel/docker/grpc.Dockerfile>`_

.. literalinclude:: ../docker/grpc.Dockerfile
   :language: dockerfile

Next, this workflow builds our devel (`//docker/devel.Dockerfile <https://github.com/farm-ng/tractor/blob/devel/docker/devel.Dockerfile>`_) image which uses the third party images like:

.. literalinclude:: ../docker/devel.Dockerfile
   :language: dockerfile
   :start-after: copy_third_party
   :end-before: copy_third_party

.. note::

    We support x86 and the jetson platform, and would like to extend this to build for platforms in the near future using:

    - https://github.com/docker/setup-buildx-action
    - https://github.com/marketplace/actions/build-and-push-docker-images#multi-platform-image
