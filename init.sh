#!/bin/bash

if [[ $EUID -ne 0 ]]; then
   echo "ОШИБКА: Запустите скрипт с sudo: sudo ./init.sh"
   exit 1
fi

usermod -a -G dialout pi
apt-get -y update --fix-missing
apt-get -y upgrade
apt-get install -y build-essential libtbb-dev pkg-config libtool autoconf automake uuid-dev
