#!/bin/bash
set -ex

# This file's path
BASEDIR="${FARM_NG_ROOT}/modules/core/tests"
# The services from docker-compose.yml that the test runner requires
TEST_DEPS="ipc_logger"
# The file that defines the test runner (and any additional override configuration)
OVERRIDE="${BASEDIR}/docker-compose.ipc_publisher_test.yml"

TMPDIR=$(mktemp -d)

docker-compose -f $BASEDIR/docker-compose.yml -f $OVERRIDE pull

_UID=`id -u` _GID=`id -g` BLOBSTORE_ROOT=$TMPDIR \
docker-compose -f $BASEDIR/docker-compose.yml -f $OVERRIDE up \
--abort-on-container-exit $TEST_DEPS test_runner

rm -rf $TMPDIR
