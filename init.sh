#!/bin/bash

sudo apt-get -y update --fix-missing
sudo apt-get -y upgrade
sudo apt-get install -y build-essential libtbb-dev
sudo apt-get install -y libtool pkg-config autoconf automake uuid-dev
