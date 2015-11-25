#!/bin/bash

set -x

sudo add-apt-repository -y ppa:hrg/daily
sudo apt-get update -qq
sudo apt-get install -qq -y choreonoid libcnoid-dev libsdformat-dev

cmake .
make

