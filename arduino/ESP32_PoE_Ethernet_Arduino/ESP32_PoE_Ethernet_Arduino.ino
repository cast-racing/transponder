
#include <ETH.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "iac_udp_struct.h"

// Static IP configuration
IPAddress local_IP(192, 168, 1, 100);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(8, 8, 8, 8);

// Globals
WiFiUDP udp;
const unsigned int localPort = 15782;  // UDP listening port
const unsigned int remotePort = 15783; // Destination UDP port
IPAddress remoteIP(192, 168, 1, 9); // Destination IP for UDP messages

const char headerA = '$';
const char headerB = 'S'; 

TransponderUdpPacket packet_in, packet_out;
static bool eth_connected = false;

// React to Ethernet events:
void WiFiEvent(WiFiEvent_t event)
{
  switch (event) {

    case ARDUINO_EVENT_ETH_START:
      // This will happen during setup, when the Ethernet service starts
      Serial.println("ETH Started");
      ETH.config(local_IP, gateway, subnet, dns);
      //set eth hostname here
      ETH.setHostname("esp32-ethernet");
      break;

    case ARDUINO_EVENT_ETH_CONNECTED:
      // This will happen when the Ethernet cable is plugged 
      Serial.println("ETH Connected");
      break;

    case ARDUINO_EVENT_ETH_GOT_IP:
    // This will happen when we obtain an IP address through DHCP:
      Serial.print("Got an IP Address for ETH MAC: ");
      Serial.print(ETH.macAddress());
      Serial.print(", IPv4: ");
      Serial.print(ETH.localIP());
      if (ETH.fullDuplex()) {
        Serial.print(", FULL_DUPLEX");
      }
      Serial.print(", ");
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      eth_connected = true;

      // Uncomment to automatically make a test connection to a server:
      // testClient( "192.168.0.1", 80 );

      break;

    case ARDUINO_EVENT_ETH_DISCONNECTED:
      // This will happen when the Ethernet cable is unplugged 
      Serial.println("ETH Disconnected");
      eth_connected = false;
      break;

    case ARDUINO_EVENT_ETH_STOP:
      // This will happen when the ETH interface is stopped but this never happens
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;

    default:
      break;
  }
}

// Try to read something from a webserver:
void testClient(const char * host, uint16_t port)
{
  Serial.print("\nConnecting to ");
  Serial.print(host);
  Serial.print(":");
  Serial.println(port);

  WiFiClient client;
  if (!client.connect(host, port)) {
    Serial.println("connection failed");
    return;
  }
  client.printf("GET / HTTP/1.1\r\nHost: %s\r\n\r\n", host);
  while (client.connected() && !client.available());
  while (client.available()) {
    Serial.write(client.read());
  }

  Serial.println("closing connection\n");
  client.stop();
}

// Initializing everything at start up / after reset:
void setup()
{
  // Wait for the hardware to initialize:
  delay(500);

  // This sketch will log some information to the serial console:
  Serial.begin(115200); // Assuming computer will be connected to serial port at 115200 bauds
  Serial.print("Setup...");

  Serial1.begin(57600);
  
  // Add a handler for network events. This is misnamed "WiFi" because the ESP32 is historically WiFi only,
  // but in our case, this will react to Ethernet events.
  Serial.print("Registering event handler for ETH events...");
  WiFi.onEvent(WiFiEvent);
  
  // Starth Ethernet (this does NOT start WiFi at the same time)
  Serial.print("Starting ETH interface...");
  ETH.begin();

  udp.begin(localPort);

}


void loop()
{
  // Serial.print("(Local) ");
  // Serial.print("ETH MAC: ");
  // Serial.print(ETH.macAddress());
  // Serial.print(", IPv4: ");
  // Serial.println(ETH.localIP());

  // Serial1.print("(Remote) ");
  // Serial1.print("ETH MAC: ");
  // Serial1.print(ETH.macAddress());
  // Serial1.print(", IPv4: ");
  // Serial1.println(ETH.localIP());

  while (Serial1.available())
  {
    Serial.print(Serial1.read());
  }

  process_udp();

  // delay(1000);
}

struct __attribute__((packed)) StructXbee
{
  char headerA = headerA;        // Header A
  char headerB = headerB;        // UTC time [ ms? ]
  TransponderUdpPacket data;     // Data
  uint8_t crc8;                  // Checksum
};

union XbeePacket
{
  StructXbee data;
  char raw[sizeof(StructXbee)];
};

const uint SIZEOF_XbeePacket = sizeof(XbeePacket);

void process_udp()
{
  // Receive UDP and send to Serial1
  char buf[100];
  
  int packetSize = udp.parsePacket();
  if (packetSize == SIZEOF_TransponderUdpPacket)
  {
    sprintf(buf,"%8d | UDP packet received (%d bytes)\n",millis(),packetSize); 
    Serial.print(buf);
    XbeePacket serial_out;

    // Copy UDP packet into Xbee struct
    int len = udp.read(serial_out.data.data, SIZEOF_TransponderUdpPacket);

    if (len == SIZEOF_TransponderUdpPacket)
    {
        // Calculate checksum for Xbee struct
        serial_out.data.crc8 = calc_crc8();

        // Forward packet out via Serial1
        Serial1.write((uint8_t*)serial_out.raw, len);
    }
    else
    {
      sprintf(buf,"Read incorrect number of bytes. Got %d, expected %d\n",len,SIZEOF_TransponderUdpPacket); 
      Serial.print("Packet size received incorrect");
    }
  }
  else
  {
    sprintf(buf,"Packet size received incorrect. Got %d, expected %d\n",packetSize,SIZEOF_TransponderUdpPacket); 
    Serial.print("Packet size received incorrect");
  }

  // Read from Serial1 and send as UDP
  while (Serial1.available())
  {
    char c = Serial1.read();
    udp.beginPacket(remoteIP, remotePort);
    udp.write(c);
    udp.endPacket();
  }
}

uint8_t calc_crc8()
{
  return 0xFF;
}
