#!/bin/bash
set -e

echo "Compiling code, please wait"
arduino-cli compile \
  --fqbn esp32:esp32:esp32-poe-iso \
  transponder/

echo "Uploading"
arduino-cli upload \
  -p /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --fqbn esp32:esp32:esp32-poe-iso \
  transponder/

echo "Done"
