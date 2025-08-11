# ArduRoomba - Clean & Minimal

A minimal, focused Arduino library for iRobot Open Interface communication.

[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/pkyanam/ArduRoomba)

## Design Philosophy

- **Minimal Core**: Essential functionality only
- **Smart Modularity**: Clean interfaces for future extensions
- **No Bloat**: Simple, readable code without over-engineering
- **Extensible**: Easy to add WiFi, Bluetooth, RTOS when needed

## Architecture

```
ArduRoomba/
├── src/
│   ├── ArduRoomba.h/.cpp          # Main library interface
│   ├── RoombaOI.h/.cpp            # Open Interface communication
│   └── extensions/                # Future: WiFi, Bluetooth, etc.
└── examples/
    ├── BasicMovement/
    ├── SensorReading/
    └── RemoteControl/
```

## Core Features

- iRobot Open Interface protocol implementation
- Basic movement commands (drive, turn, stop)
- Sensor reading (individual and streaming)
- Simple LED and sound control
- Clean error handling (bool returns, no complex enums)

## Future Extensions

- WiFi module (ESP32/ESP8266)
- Bluetooth control
- RTOS integration
- Advanced sensors
- Custom behaviors

## Usage

```cpp
#include "ArduRoomba.h"

ArduRoomba roomba(2, 3, 4); // RX, TX, BRC pins

void setup() {
  Serial.begin(19200);
  if (roomba.begin()) {
    Serial.println("Roomba connected!");
    roomba.moveForward();
    delay(2000);
    roomba.stop();
  }
}
```

## Supported Hardware

- Arduino Uno R3/R4
- ESP32/ESP8266
- iRobot Create 2, Roomba 500/600/700 series

## Dev Installation

## Installation

Using docker:

```bash
docker build -t roomba docker --label roomba.arduino=true
docker run -it --device=/dev/ttyUSB0 -v $PWD/examples:/root/sketch -v $PWD/:/root/Arduino/libraries/ArduRoomba -v /tmp:/tmp roomba bash
```

## Usage
### ESP8266 side

First go to an example ino project (not for docker usage):
```bash
cd BasicMovement
```

```bash
roomba-compile
```
Allow your user to read and write your COM port (not for docker usage):

```bash
sudo chmod a+rw /dev/ttyUSB0
```

Upload the compiled program to the board :

```bash
roomba-upload
```

For listennig COM port (ESP8266 log server) :

```bash
tail -f /tmp/esp8266-received.log | xargs -IL date +"%Y-%m-%d %H:%M:%S L"
```

```bash
tail -f /tmp/esp8266-received.log | grep -a --line-buffered "FREEHEAP"
```

if the file `/tmp/esp8266-received.log` does not exist or you do not read any log, then run the command : 

```bash
roomba-logtty
```
