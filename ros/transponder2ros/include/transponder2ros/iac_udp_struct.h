
// Transponder Structs

#include <stdint.h>

const uint8_t TRANSPONDER_UDP_STRUCT_VERISON = 0x01;  // 2025-02-21
                                                      // Check on the ROS side the versions of the
                                                      // structs you are getting are correct
enum class VehicleStateTransponder
{
  UNKNOWN,
  EMERGENCY_STOP,
  CONTROLLED_STOP,
  NOMINAL
};

struct __attribute__((packed)) StructIacTransponder
{
  uint8_t version;               // Struct version
  double utc;                    // UTC time [ s ]
  uint8_t car_id;                // Car ID [ - ]
  double lat;                    // Vehicle longitude [ dd.dd ]
  double lon;                    // Vehicle latitude [ dd.dd ]
  float vel;                     // Vehicle speed [ m/s ]
  VehicleStateTransponder state; // Vehicle state [ - ]
};

union TransponderUdpPacket
{
  StructIacTransponder data;
  char raw[sizeof(StructIacTransponder)];
};

const uint SIZEOF_TransponderUdpPacket = sizeof(TransponderUdpPacket);
