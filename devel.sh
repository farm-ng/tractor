#!/bin/bash -ex
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
COMMAND=$@
make -C docker/devel upd
docker exec devel_workspace_1 $COMMAND
