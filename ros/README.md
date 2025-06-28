# ROS2 Node
There are two nodes that are required to use the transponder system
- a node to converts the best estimate of the state (`odom2transponder_node`) into a `transponder_msgs::Transponder.msg`
  - `ros/transponder2ros/src/odom2transponder_node.cpp` converts odom messages into `transponder_msgs::Transponder.msg`.
  - Requires the base lat/lon/alt to be set correctly to convert ENU to lla.
  - For advanced users, you can also fill the car state to act as a virtual 'brake lights.' 
- a node to broadcast/receive these messages (`transponder2ros`) via the transponder. 
  - `ros/transponder2ros/src/transponder2ros.cpp` broadcasts and receives `transponder_msgs`
  - Requires accurate time synchronisation between transponders.  It is highly recommended to synchonise you computer with the ntp servers `sudo ntpdate -u pool.ntp.org` when starting your code stack to reduce the chance your data will be rejected.

Launching these nodes should be plug and play if you update `ros/transponder2ros/launch/transponder.launch.py` correctly.