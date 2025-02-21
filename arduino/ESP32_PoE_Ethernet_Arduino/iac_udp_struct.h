
#include <stdint.h>

enum class VehicleStateTransponder
{
  UNKNOWN,
  EMERGENCY_STOP,
  CONTROLLED_STOP,
  NOMINAL
};

union struct_iacTransponder
{
  uint64_t utc;                  // UTC time [ ms? ]
  uint8_t car_id;                // Car ID [ - ]
  double lat;                    // Vehicle longitude [ dd.dd ]
  double lon;                    // Vehicle latitude [ dd.dd ]
  float vel;                     // Vehicle speed [ m/s ]
  VehicleStateTransponder state; // Vehicle state [ - ]
};

