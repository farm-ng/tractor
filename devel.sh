#!/bin/bash -ex
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
TAG=farmng/devel@sha256:ae2c7f801fcd8ffda5ca1e3f24321f3eef5962b81fb1d57026816aca22e270e7

COMMAND=$@
docker run \
       --rm -v $DIR:/workspace/tractor:rw,Z --workdir /workspace/tractor \
        $TAG ./env.sh $COMMAND
