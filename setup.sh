#!/bin/bash

cp Python/Integration/Bigfoot.py Python

# Setup Arduino
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
f=$(pwd)
export PATH=$PATH:$f/bin/
sudo cp $f/bin/arduino-cli /bin
arduino-cli core install arduino:avr
arduino-cli board list

# Setup the I2C / Bigfoot Libraries
sudo pip3 install smbus
sudo pip3 install board
sudo pip3 install adafruit-circuitpython-ina219

# BOOT UP to GUI
sudo mkdir /home/microdev/.config/autostart/
sudo cp MyApp.desktop1 /home/microdev/.config/autostart/MyApp.desktop
cd /home/microdev/.config/autostart/
sudo chmod +x /home/microdev/.config/autostart/MyApp.desktop