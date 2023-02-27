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

# install systemd service to enable on reboot
sudo sh -c "echo '[Unit]
Description=Startup service for linorobot 2 stack
After=network.target
StartLimitIntervalSec=0

[Service]
Type=simple
Restart=always
RestartSec=1
User=ubuntu
ExecStart=/usr/bin/sudo /usr/bin/bash /home/ubuntu/P10/bringup.sh

[Install]
WantedBy=multi-user.target' >> /lib/systemd/system/bringup.service"

sudo systemctl daemon-reload
sudo systemctl enable bringup

echo "Installed bringup service, restart the pi, or run: 'systemctl start bringup' in order to start the bringup service"