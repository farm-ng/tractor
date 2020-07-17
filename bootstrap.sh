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

sudo apt-get update
sudo apt-get install -y \
     python3-pip

sudo pip3 install virtualenv

virtualenv $FARM_NG_ROOT/env
. $FARM_NG_ROOT/env/activate

pip install -r $FARM_NG_ROOT/requirements.txt




