
// Transponder Structs

#include <stdint.h>

const uint8_t TRANSPONDER_UDP_STRUCT_VERISON = 0x02;  // 2025-02-21
                                                      // Check on the ROS side the versions of the
                                                      // structs you are getting are correct

struct __attribute__((packed)) StructIacTransponder
{
  uint8_t version;               // Struct version
  double utc;                    // UTC time [ s ]
  uint8_t car_id;                // Car ID [ - ]
  double lat;                    // Vehicle longitude [ dd.dd ]
  double lon;                    // Vehicle latitude [ dd.dd ]
  float vel;                     // Vehicle speed [ m/s ]
  uint8_t state;                 // Vehicle state [ - ], see transponder_msgs::msg::Transponder
};

union TransponderUdpPacket
{
  StructIacTransponder data;
  char raw[sizeof(StructIacTransponder)];
};

const uint SIZEOF_TransponderUdpPacket = sizeof(TransponderUdpPacket);
