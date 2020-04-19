#!/bin/bash
# Get directory where this script is located
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  # if $SOURCE was a relative symlink, we need to resolve it
  # relative to the path where the symlink file was located
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
done

SETUP="$( dirname "${SOURCE}" )"/setup.bash
RCFILE="$( dirname "${SOURCE}" )"/bashrc

COMMAND=$@
if [[ -z "$COMMAND" ]] ; then
    COMMAND="bash --rcfile $RCFILE"
fi
source $SETUP
$COMMAND


