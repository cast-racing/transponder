# Transponder

Transponder between cars for the IAC.

![image](images/electronics.jpg)

Features
- Arduino based
- ROS2 node example
- Powered via PoE

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
