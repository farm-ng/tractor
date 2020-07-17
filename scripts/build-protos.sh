#!/bin/bash
set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
CMD="protoc"
CMD_ARGS="--proto_path=protos
          --python_out=python/genproto
          --ts_proto_out=app/frontend/genproto
          --ts_proto_opt=forceLong=long
          --twirp_tornado_srv_out=python/gensrv
          protos/farm_ng_proto/tractor/v1/*.proto"

TAG="protoc"

docker build -t $TAG -f $DIR/Dockerfile.protoc $DIR
docker run \
       --rm -v $PWD:/src:rw,Z -u $(id -u):$(id -g) --workdir /src \
       --entrypoint $CMD $TAG $CMD_ARGS
