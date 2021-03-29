#!/bin/bash -e
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
version=$(cd $DIR && git describe --tags --match "v*" || echo v0.11.3)
version=${version:1}
echo ${version//[-]*}
