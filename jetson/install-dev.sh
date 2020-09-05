#!/bin/bash -ex
. ../setup.bash

# Eventually we won't be building these artifacts on the device, but for now they're necessary.

# Go
wget https://golang.org/dl/go1.15.1.linux-arm64.tar.gz -P /tmp/
sudo tar -C /usr/local -xzf /tmp/go1.15.1.linux-arm64.tar.gz
/usr/local/go/bin/go version

# Node
sudo apt update
sudo apt -y install curl dirmngr apt-transport-https lsb-release ca-certificates
curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
sudo apt -y install nodejs
nodejs --version

# Yarn
curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | sudo apt-key add -
echo "deb https://dl.yarnpkg.com/debian/ stable main" | sudo tee /etc/apt/sources.list.d/yarn.list
sudo apt update
sudo apt -y install yarn

cd $FARM_NG_ROOT && make protos && make frontend
