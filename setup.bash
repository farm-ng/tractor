# Get directory where this script is located
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  # if $SOURCE was a relative symlink, we need to resolve it
  # relative to the path where the symlink file was located
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
done
export FARM_NG_ROOT=$( cd "$( dirname "${SOURCE}" )" >/dev/null 2>&1 && pwd )

. $FARM_NG_ROOT/env/bin/activate

export PYTHONPATH=$FARM_NG_ROOT/python:$FARM_NG_ROOT/python/genproto:$FARM_NG_ROOT/env/lib
export LD_LIBRARY_PATH=$FARM_NG_ROOT/env/lib
export PATH=$PATH:/usr/local/go/bin
