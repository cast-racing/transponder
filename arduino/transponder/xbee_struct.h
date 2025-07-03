
#include <stdint.h>

struct __attribute__((packed)) StructXbee
{
  char headerA;                 // Header A
  char headerB;                 // UTC time [ ms? ]
  TransponderUdpPacket data;    // Data
  uint8_t crc8;                 // Checksum
};

union XbeePacket
{
  StructXbee data;
  char raw[sizeof(StructXbee)];
};

const uint SIZEOF_XbeePacket = sizeof(XbeePacket);
