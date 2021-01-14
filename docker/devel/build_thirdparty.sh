#!/bin/bash -ex

docker build --memory=8024m --build-arg PARALLEL=$(nproc --ignore=2) -f $1.Dockerfile -t farmng/build-$1:latest -t farmng/build-$1:`git rev-parse --short HEAD` ./
