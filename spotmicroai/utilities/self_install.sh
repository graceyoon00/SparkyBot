#!/bin/bash

sudo apt update -y

sudo apt dist-upgrade -y

sudo apt autoremove -y

sudo apt install git python3-venv sshpass i2c-tools python-smbus joystick xboxdrv -y

grep -qxF 'options bluetooth disable_ertm=Y' /etc/modprobe.d/bluetooth.conf || echo 'options bluetooth disable_ertm=Y' | sudo tee -a /etc/modprobe.d/bluetooth.conf
cat /etc/modprobe.d/bluetooth.conf

cd ~/SparkyBot || exit
git clone https://gitlab.com/custom_robots/spotmicroai/basic-runtime.git spotmicroai
cd spotmicroai || exit

find . -type f -iname "*.sh" -exec chmod +x {} \;

~/SparkyBot/spotmicroai/utilities/activate.sh