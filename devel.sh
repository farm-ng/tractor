#!/bin/bash -e
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
make -C docker/devel upd > /dev/null
bash_args=$@
if [[ -z "$bash_args" ]] ; then
    bash_args="bash"
fi
test -t 1 && USE_TTY="-t"
set -x
docker exec -i $USE_TTY devel_workspace_1 bash -c "$bash_args"
