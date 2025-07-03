# ROS2 Node
There are two nodes that are required to use the transponder system
- a node to converts the best estimate of the state (`odom2transponder_node`) into a `transponder_msgs::Transponder.msg`
  - `ros/transponder2ros/src/odom2transponder_node.cpp` converts odom messages into `transponder_msgs::Transponder.msg`.  See [Transponder.msg](transponder_msgs/msg/Transponder.msg) for more information.
  - Requires the base lat/lon/alt to be set correctly to convert ENU to lla.
  - For advanced users, you can also fill the car state to act as a virtual 'brake lights.' Keep this is `transponder_msgs::msg::Transponder::STATE_UNKNOWN` if you don't intend on filling it.
- a node to broadcast/receive these messages (`transponder2ros_node`) via the transponder. 
  - `ros/transponder2ros/src/transponder2ros.cpp` broadcasts and receives `transponder_msgs`
  - Requires accurate time synchronisation between transponders.  It is highly recommended to synchonise you computer with the ntp servers `sudo ntpdate -u pool.ntp.org` when starting your code stack to reduce the chance your data will be rejected.
  - This will broadcast all messages received so make sure to keep your publish rate of the `Transponder.msg` to a reasonable rate (e.g. 10 Hz) to not flood the network.

Launching these nodes should be plug and play if you update `ros/transponder2ros/launch/transponder.launch.py` correctly.

## Notes
### Altitude
A GPS can provide the altitude in either ellipsoid or geoid height.
The transponder system expects ellipsoid, so the user must check they are providing the correctly datumed information.

The Novatel OEM7 receivers use ellipsoid and can be directly fed into the system.
However, the VN310 uses geoid, so this must be offset by the user before transmission.
The geoid / ellipsoid offset is different around the world, so use the [UNAVCO Calcuator](https://www.unavco.org/software/geodetic-utilities/geoid-height-calculator/geoid-height-calculator.html) to find the offset for the current location.

As an example, for Laguna Seca
- The Pit lat/lon is (36.58765756, -121.75527943)
  - From the UNAVCO Calculator, the geoid height is -33.76 m   
- The altitude above the ellipsoid is 232.556 m (as reported by the Novatel OEM7)
- The VN310 reads 199.563 m (the height above the geoid)
  - This altitude must be converted to ellpisoid by subtracting the -33.76 m offset
  - 199.563 - (-33.76) = 233.323
  - Broadcast the ellipsoid height of 233.323 m
