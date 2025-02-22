# ROS message to publish who is in command

std_msgs/Header header  # Standard ROS header

uint8 car_id            # Car ID [ - ]
float64 lat             # Vehicle longitude [ dd.dd ]
float64 lon             # Vehicle latitude [ dd.dd ]
float32 vel             # Vehicle speed [ m/s ]
uint8  state            # Vehicle state, see iac_udp_struct.h [ - ]

# Vehicle states
uint8 STATE_UNKNOWN = 0
uint8 STATE_EMERGENCY_STOP = 1
uint8 STATE_CONTROLLED_STOP = 2
uint8 STATE_NOMINAL = 3
