#!/bin/bash -ex
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
TAG=farmng/devel@sha256:0be3c6a66403711dd76fac557d5c73c4fb626bcd74795a106ab65e1b251e0c6d

COMMAND=$@
docker run \
       --rm -v $DIR:/workspace/tractor:rw,Z --workdir /workspace/tractor \
        $TAG ./env.sh $COMMAND
