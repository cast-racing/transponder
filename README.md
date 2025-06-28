# Transponder

Transponder between cars for the IAC.

Features
- Arduino based
- ROS2 node example
- Powered via PoE


## Theory of Operation
The ESP32-PoE-ISO listens to UDP packets published from the ROS2 node `transponder2ros_node`.
These are then packaged with a header and a checksum and sent out as serial data over the xbee network to all the available nodes.
The ESP32-PoE-ISO takes any received packets over the xbee network, checks their integrity, and converts them back into UDP packets, and forwards them back to the `transponder2ros_node`.
The node then publishes the data out as a `transponder_msgs::msg::Transponder` onto the ROS2 network.

Each air packet is about 40 bytes long, so the network can handle about `XXXXX, to test in real world conditions` packets/sec between all nodes.

## Installation
### ROS2
All dependencies on the ROS2 side can be installed via rosdep.
From your main workspace folder, run `rosdep install --from-paths src --ignore-src -r -y`.
If you have trouble with finding geographiclibs, debian/ubuntu installs `FindGeographicLib.cmake` in a non-standard location, but this should be covered in the `CMakeLists.txt` file.

### Arduino
The Arduino code is only required to update the transponder firmware.
See the [Arduino README.md](arduino/README.md) for installation instructions.

### XBee
[XTCU](https://www.digi.com/products/embedded-systems/digi-xbee/digi-xbee-tools/xctu) is used to configure the XBee.  This should only be required once.

## Component Setup
See the individual README.md files in each folder for more information - [ESP32-PoE-ISO (Arduino)](arduino/README.md) / [ROS](ros/README.md) / [XBee](xbee/README.md).

## Bill of Materials
- Compute
  - Olimex ESP32-PoE-ISO
    - https://www.digikey.com/en/products/detail/olimex-ltd/ESP32-POE-ISO/10258716
- Radio
  - Xbee radio
    - https://www.digikey.com/en/products/detail/digi/XB3-24Z8ST/8130935
  - Xbee breakout
    - https://www.digikey.com/en/products/detail/sparkfun-electronics/BOB-08276/5318741
  - Xbee headers
    - https://www.digikey.com/en/products/detail/adafruit-industries-llc/366/5629440
  - Xbee to USB breakout (recommended for programming Xbees)
    - https://www.digikey.com/en/products/detail/sparkfun-electronics/11812/5762455
- Antennas
  - Antenna (small)
    - https://www.digikey.com/en/products/detail/te-connectivity-linx/ANT-2-4-LCW-RPS/9607800
  - Antenna (large)
    - https://www.digikey.com/en/products/detail/taoglas-limited/FW-24-SMA-M/7035228
- Extension Cables
  - 250 mm (9.8") RP-SMA to RP-SMA
    - https://www.digikey.com/en/products/detail/sparkfun-electronics/22037/21443079
  - 1 m (3'3") RP-SMA to RP-SMA
    - https://www.digikey.com/en/products/detail/sparkfun-electronics/22036/21443073

## Wiring
![image](images/electronics.jpg)
