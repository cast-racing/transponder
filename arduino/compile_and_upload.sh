#!/bin/bash
# Uses arduino-cli : curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
# Install board support : arduino-cli core update-index && arduino-cli core install esp32:esp32

# arduino-cli board search olimex

set -e

echo "Compiling code"
arduino-cli compile --fqbn esp32:esp32:esp32-poe-iso transponder/ 

echo "Uploading"
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32-poe-iso transponder/

echo "Done"
