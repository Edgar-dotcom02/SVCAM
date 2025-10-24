#!/bin/bash

echo "=== Установка dxlSlave.service для SVCAM ==="

if [[ $EUID -ne 0 ]]; then
   echo "ОШИБКА: Запустите скрипт с sudo: sudo ./install_dxl.sh"
   exit 1
fi

mkdir -p /usr/bin/dxl

if [[ ! -f "DxlSlave" ]]; then
    echo "ОШИБКА: Файл DxlSlave не найден!"
    echo "Проверьте, что вы находитесь в папке dxlSlaveService"
    exit 1
fi

cp DxlSlave /usr/bin/dxl/
cp config.json /usr/bin/dxl/
cp libSystem.IO.Ports.Native.so /usr/bin/dxl/
chmod +x /usr/bin/dxl/DxlSlave

cp dxlSlave.service /etc/systemd/system/

systemctl daemon-reload
systemctl enable dxlSlave.service
systemctl start dxlSlave.service

# Проверка статуса
if systemctl is-active --quiet dxlSlave.service; then
    echo "=== УСТАНОВКА УСПЕШНА! ==="
fi
