#!/bin/bash -e
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

FARM_NG_DEVEL_IMAGE=${FARM_NG_DEVEL_IMAGE:-"farmng/devel@$(cat $DIR/docker/devel/devel-digest.txt)"}
BLOBSTORE_ROOT=${BLOBSTORE_ROOT:-$DIR/../tractor-data}
make FARM_NG_DEVEL_IMAGE=$FARM_NG_DEVEL_IMAGE \
     BLOBSTORE_ROOT=$BLOBSTORE_ROOT \
     -C $DIR/docker/devel upd
bash_args=$@
if [[ -z "$bash_args" ]] ; then
    bash_args="bash"
fi
test -t 1 && USE_TTY="-t"
set -x
docker exec -i $USE_TTY devel_workspace_1 bash -c "$bash_args"
