#!/bin/bash -ex

docker build --memory=8024m --build-arg PARALLEL=$(nproc --ignore=2) -f base.Dockerfile -t farmng/base:latest -t farmng/base:`git rev-parse --short HEAD` ../
