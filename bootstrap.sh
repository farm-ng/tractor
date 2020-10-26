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

$FARM_NG_ROOT/bootstrap-apt.sh
$FARM_NG_ROOT/bootstrap-bin.sh
$FARM_NG_ROOT/bootstrap-venv.sh


# tractor-logs
if [ ! -d "$HOME/tractor-logs" ]; then
  if git clone git@github.com:farm-ng/tractor-logs.git $HOME/tractor-logs; then
    cd ~/tractor-logs
    git lfs install --local
    git config user.email `hostname`
    git config user.name `hostname`
  else
    echo "Please generate an SSH keypair and add it to the tractor-logs repository."
    exit 1
  fi
fi
