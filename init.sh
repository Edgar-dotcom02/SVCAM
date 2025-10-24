#!/bin/bash

echo "=== Обновление пакет программ для SVCAM ==="

if [[ $EUID -ne 0 ]]; then
   echo "ОШИБКА: Запустите скрипт с sudo: sudo ./init.sh"
   exit 1
fi

usermod -a -G dialout pi
apt-get -y update --fix-missing
apt-get -y upgrade
apt-get install -y build-essential libtbb-dev pkg-config libtool autoconf automake uuid-dev

echo "=== Обновление ОКОНЧЕНО ==="
echo "Пожалуйста перезагрузите компьютер"
