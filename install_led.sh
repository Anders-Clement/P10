#!/usr/bin/env bash

# small script to install and start pigpio driver for gpio access

echo "Make sure you have not run this script before. Proceed?"
echo -n "Yes [y] or No [n]: " 
read reply
if [[ "$reply" == "n" || "$reply" == "N" ]]
    then
        echo "Exiting..."
        exit 0
fi

sudo apt install python3-pigpio

cd src
wget https://github.com/joan2937/pigpio/archive/master.zip
sudo apt install unzip
unzip master.zip
cd pigpio-master
make
sudo make install
cd ..
rm master.zip

# install systemd service to enable on reboot
sudo sh -c "echo '[Unit]
Description=Daemon required to control GPIO pins via pigpio
[Service]
ExecStart=/usr/local/bin/pigpiod
ExecStop=/bin/systemctl kill -s SIGKILL pigpiod
Type=forking
[Install]
WantedBy=multi-user.target' >> /lib/systemd/system/pigpiod.service"

sudo systemctl daemon-reload
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

echo "You will need to restart the pi, or run: 'sudo pigpiod'. Following reboots will autostart pigpiod"