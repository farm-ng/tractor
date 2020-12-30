# Get directory where this script is located
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  # if $SOURCE was a relative symlink, we need to resolve it
  # relative to the path where the symlink file was located
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
done
export FARM_NG_PREFIX=$( cd "$( dirname "${SOURCE}" )" >/dev/null 2>&1 && pwd )/env

export PYTHONPATH=$FARM_NG_PREFIX/lib/python3.6/site-packages:$FARM_NG_PREFIX/lib/python3.6/dist-packages
export LD_LIBRARY_PATH=$FARM_NG_PREFIX/env/lib

FARM_NG_GOPATH=$FARM_NG_PREFIX/go
export GOPATH=$FARM_NG_GOPATH:$GOPATH
export PATH=$FARM_NG_GOPATH/bin:/usr/local/go/bin:$PATH
