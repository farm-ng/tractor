#!/bin/bash -ex
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
COMMAND=$@
uid=`id -u` gid=`id -g` docker-compose -f docker/devel/docker-compose.yml run --rm workspace $COMMAND
