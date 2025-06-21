# Transponder

Transponder between cars for the IAC.

![image](images/electronics.jpg)

Features
- Arduino based
- ROS2 node example
- Powered via PoE

## Installation
All dependencies on the ROS2 side can be installed via rosdep.
From your main workspace folder, run `rosdep install --from-paths src --ignore-src -r -y`.
If you have trouble with finding geographiclibs, debian/ubuntu installs `FindGeographicLib.cmake` in a nonstand location, but this should be covered in the `CMakeLists.txt` file.

## Theory of Operation
The ESP32-PoE-ISO listens to UDP packets published from the ROS2 node `ros2 run transponder2ros transponder2ros_node`.
These are then packaged with a header and a checksum and sent out as serial data over the xbee network to all the available nodes.
The ESP32-PoE-ISO takes any received packets over the xbee network, checks their integrity, and converts them back into UDP packets, and forwards them back to the `transponder2ros_node`.
The node then publishes the data out as a `transponder_msgs::msg::Transponder` onto the ROS2 network.

## Components
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
- Antennas
  - Antenna (small)
    - https://www.digikey.com/en/products/detail/te-connectivity-linx/ANT-2-4-LCW-RPS/9607800
  - Antenna (large)
    - https://www.digikey.com/en/products/detail/taoglas-limited/FW-24-SMA-M/7035228
- Extension Cables (should be correct)
  - 250 mm (9.8") RP-SMA to RP-SMA
    - https://www.digikey.com/en/products/detail/sparkfun-electronics/22037/21443079
  - 1 m (3'3") RP-SMA to RP-SMA
    - https://www.digikey.com/en/products/detail/sparkfun-electronics/22036/21443073

## Component Setup
### ROS2 Node
To fill in with a better example, but the general idea is
- `ros/transponder2ros/src/odom2transponder_node.cpp` converts odom messages into `transponder_msgs`
- `ros/transponder2ros/src/transponder2ros.cpp` broadcasts and receives the `transponder_msgs`
It should be mostly plug and play if you update `ros/transponder2ros/launch/transponder.launch.py` correctly

### ESP32-PoE-ISO
Software can be flashed using Arduino
- Change the network settings in `arduino/transponder/transponder.ino` to match your network
  - `local_IP` is the address of the transponder (same subnet as the PoE switch)
  - `ip_send_` should be the IP address of the PoE switch 
- Upload as `OLIMEX ESP32-POE-ISO` board type.  Requires the [Arduino-ESP32 package](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html). 

### XBee
1. Download XCTU, and apply the profile in `xbee/iac_default.pro`.
1. Power cycle the XBee.  The baud rate will be switched to 57600 baud.
1. Connect again and set the Node ID to the car number
