#!/bin/bash -ex

# Go
arch=`dpkg --print-architecture`
if ! /usr/local/go/bin/go version | grep 1.15.1; then
  wget https://golang.org/dl/go1.15.1.linux-${arch}.tar.gz -P /tmp/
  sudo tar -C /usr/local -xzf /tmp/go1.15.1.linux-${arch}.tar.gz
  /usr/local/go/bin/go version
fi

# Prometheus Node Exporter
if ! node_exporter --version | grep 1.0.1; then
  wget https://github.com/prometheus/node_exporter/releases/download/v1.0.1/node_exporter-1.0.1.linux-${arch}.tar.gz -P /tmp/
  tar -C /tmp -xzf /tmp/node_exporter-1.0.1.linux-${arch}.tar.gz
  sudo cp /tmp/node_exporter-1.0.1.linux-${arch}/node_exporter /usr/local/bin
fi

# Prometheus
if ! prometheus --version | grep 2.21.0; then
  wget https://github.com/prometheus/prometheus/releases/download/v2.21.0/prometheus-2.21.0.linux-${arch}.tar.gz -P /tmp/
  tar -C /tmp -xzf /tmp/prometheus-2.21.0.linux-${arch}.tar.gz
  sudo cp /tmp/prometheus-2.21.0.linux-${arch}/prometheus /usr/local/bin
fi
