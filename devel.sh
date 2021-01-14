#!/bin/bash -ex
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
TAG=farmng/devel@sha256:15c31ddbeb3649575de3dfbc4c3360c1b5af04e04fbc275fd7c9de4cde60b89d

COMMAND=$@
docker run \
       --rm -v $DIR:/workspace/tractor:rw,Z --workdir /workspace/tractor \
        $TAG ./env.sh $COMMAND
