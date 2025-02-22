
#include <stdint.h>

const char xbee_headerA_ = '$';
const char xbee_headerB_ = 'S'; 

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
