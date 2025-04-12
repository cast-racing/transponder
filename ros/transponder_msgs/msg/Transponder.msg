# ROS message to publish who is in command

std_msgs/Header header  # Standard ROS header

uint8 car_id            # Car ID [ - ]
float64 lat             # Vehicle longitude, centre of rear axle [ dd.dd ]
float64 lon             # Vehicle latitude, centre of rear axle [ dd.dd ]
float32 heading         # Vehicle heading, GPS style, North = 0, East = 90 [ deg ]
float32 vel             # Vehicle speed, Vx_body [ m/s ]
uint8  state            # Vehicle state, see iac_udp_struct.h [ - ]

# Vehicle states
uint8 STATE_UNKNOWN = 0
uint8 STATE_EMERGENCY_STOP = 1
uint8 STATE_CONTROLLED_STOP = 2
uint8 STATE_NOMINAL = 3
