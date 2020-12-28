#!/bin/bash -ex
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  # if $SOURCE was a relative symlink, we need to resolve it
  # relative to the path where the symlink file was located
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
done
export FARM_NG_ROOT=$( cd "$( dirname "${SOURCE}" )" >/dev/null 2>&1 && pwd )

# Python
if [ ! -f $FARM_NG_ROOT/env/bin/activate ]; then
    python3 -m venv $FARM_NG_ROOT/env
fi

. $FARM_NG_ROOT/env/bin/activate
pip3 install --upgrade pip
pip3 install -r $FARM_NG_ROOT/requirements.txt

# Go
FARM_NG_GOPATH=$FARM_NG_ROOT/env/go
mkdir -p $FARM_NG_GOPATH
export GOPATH=$FARM_NG_GOPATH:$GOPATH
go get -u github.com/golang/protobuf/protoc-gen-go
go get -u github.com/twitchtv/twirp/protoc-gen-twirp
