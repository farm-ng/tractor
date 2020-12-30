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

# Some pre-commit checks not supported on ARM
if [ `dpkg --print-architecture` == "arm64" ]; then
  export SKIP=proto-lint
fi

for module in core calibration frontend perception tractor
do
  export PYTHONPATH=$FARM_NG_ROOT/modules/$module/python:$PYTHONPATH
  export PYTHONPATH=$FARM_NG_ROOT/build/modules/$module/protos/python:$PYTHONPATH
done

# Set BLOBSTORE_ROOT if it's not already set
: ${BLOBSTORE_ROOT:=`dirname $FARM_NG_ROOT`/tractor-data}
export BLOBSTORE_ROOT
