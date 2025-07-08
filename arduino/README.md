# Transponder (Arduino)

The ESP32-PoE-ISO is programmed using Arduino, and is required to set up the IP addresses.  Flashing should only be required once and then only during message version updates.

## Installation
### Linux
- Download Arduino CLI (or use an existing Arduino install)
  - Follow the instructions at https://arduino.github.io/arduino-cli/1.2/installation/, or,
  - `curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh`
    - Move the file `bin/arduino-cli` to `/usr/bin`
- Install board support (search cores with  `arduino-cli board search olimex`)
  - `arduino-cli core update-index`
  - `arduino-cli config set network.connection_timeout 600s`
  - `arduino-cli core install esp32:esp32`

### Windows
Windows doesn't support arduino-cli, so there's a little more manual setup requried.
- Download Arduino IDE from https://www.arduino.cc/en/software
- Install the ESP32 Core
  - `Tools > Board > Board Manager` and search for esp32
  - Install `esp32 by Espressif Systems`
- Under `Select Board > Select Other Board and Port`
  -  Boards: `OLIMEX ESP32-POE-ISO`
  -  Ports: Select the correct port for the Transponder (plugged in via USB)
-  Modify the code as below, then upload using the arrow in the circle (top left-hand corner near the tick).

You can see the debug messages through the Serial Monitor
- `Tools > Serial Monitor`

## Flashing the Firmware
- Change the network settings in `arduino/transponder/transponder.ino` to match your network
  - Comment / uncomment the `#define` at the top of the script to match your setup
    - `#define TRANSPONDER_IP "10.42.7.61"` is the IP address of your transponder
    - `#define COMPUTER_IP "10.42.7.4"` is the IP address of your computer's network interface
    - If your car is not listed, submit a PR to get it added
- Upload using the script `./compile_and_upload`.  Check the devices match (defaults to `/dev/ttyUSB0`)
  - Alternatively use the Arduino IDE and upload as `OLIMEX ESP32-POE-ISO` board type.
    - Requires the [Arduino-ESP32 package](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html).
- The USB connection provides debug information at a baud rate of 115200 including IPs and received information.

## Debugging
The main way to debug is to get the transponder to print when it receives packets from either the UDP or XBee ports.
These messages can be viewed by connecting to the uUSB port on the transponder and connecting via a serial port at 115200 baud.
On a heavily loaded network, the debugging will likely overload the serial port, so the messages are disabled by default.
To enable the debug messages, set `const bool print_debug_ = 0;` to `1` in `transponder.ino`, an re-upload the firmware.
