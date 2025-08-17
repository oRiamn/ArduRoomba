/**
 * @file RoombaOI.cpp
 * @brief Implementation of iRobot Open Interface communication
 */

#include "RoombaOI.h"

namespace ArduRoomba {

RoombaOI::RoombaOI(uint8_t rxPin, uint8_t txPin, uint8_t brcPin)
  : _rxPin(rxPin), _txPin(txPin), _brcPin(brcPin), _connected(false), _debug(false) {
  _serial = new SoftwareSerial(rxPin, txPin);
}

bool RoombaOI::begin(uint32_t baudRate) {
  if (_connected) return true;
  
  // Setup BRC pin
  pinMode(_brcPin, OUTPUT);
  digitalWrite(_brcPin, HIGH);
  
  debugPrint("Initializing Roomba OI...");
  
  // Wait for power stabilization
  delay(2000);
  // Pulse BRC to wake up
  pulseDD();

  delay(150);
  
  // Start serial communication
  _serial->begin(baudRate);
  delay(150);
  
    _connected = true;
  // Send start command
  start();
  delay(150);
  
  // Enter safe mode
  safeMode();
  delay(150);
  
  uint8_t mode = getMode();
  if(mode != 3) { // why "3" ? safe mode should be "2" 
    debugPrint("Test Mode failed", mode);
    _connected = false;
    return false;
  }
  debugPrint("Roomba OI initialized successfully");
  return true;
}

void RoombaOI::end() {
  if (_connected) {
    powerOff();
    _serial->end();
    _connected = false;
  }
}

void RoombaOI::start() {
  sendCommand(OI_START);
  debugPrint("START command sent");
}

void RoombaOI::safeMode() {
  sendCommand(OI_SAFE);
  debugPrint("SAFE mode command sent");
}

void RoombaOI::fullMode() {
  sendCommand(OI_FULL);
  debugPrint("FULL mode command sent");
}

void RoombaOI::powerOff() {
  sendCommand(OI_POWER);
  debugPrint("POWER OFF command sent");
}

void RoombaOI::drive(int16_t velocity, int16_t radius) {
  // Clamp velocity to valid range
  if (velocity > MAX_VELOCITY) velocity = MAX_VELOCITY;
  if (velocity < MIN_VELOCITY) velocity = MIN_VELOCITY;
  
  uint8_t params[4];
  params[0] = (velocity >> 8) & 0xFF;  // High byte
  params[1] = velocity & 0xFF;         // Low byte
  params[2] = (radius >> 8) & 0xFF;    // High byte
  params[3] = radius & 0xFF;           // Low byte
  
  sendCommand(OI_DRIVE, params, 4);
  debugPrint("DRIVE command", velocity);
}

void RoombaOI::driveDirect(int16_t rightVel, int16_t leftVel) {
  // Clamp velocities
  if (rightVel > MAX_VELOCITY) rightVel = MAX_VELOCITY;
  if (rightVel < MIN_VELOCITY) rightVel = MIN_VELOCITY;
  if (leftVel > MAX_VELOCITY) leftVel = MAX_VELOCITY;
  if (leftVel < MIN_VELOCITY) leftVel = MIN_VELOCITY;
  
  uint8_t params[4];
  params[0] = (rightVel >> 8) & 0xFF;  // Right high byte
  params[1] = rightVel & 0xFF;         // Right low byte
  params[2] = (leftVel >> 8) & 0xFF;   // Left high byte
  params[3] = leftVel & 0xFF;          // Left low byte
  
  sendCommand(OI_DRIVE_DIRECT, params, 4);
  debugPrint("DRIVE_DIRECT command");
}

void RoombaOI::clean() {
  sendCommand(OI_CLEAN);
  debugPrint("CLEAN command sent");
}

void RoombaOI::spot() {
  sendCommand(OI_SPOT);
  debugPrint("SPOT command sent");
}

void RoombaOI::seekDock() {
  sendCommand(OI_SEEK_DOCK);
  debugPrint("SEEK_DOCK command sent");
}

void RoombaOI::setMotors(bool mainBrush, bool sideBrush, bool vacuum) {
  uint8_t motorBits = 0;
  if (sideBrush) motorBits |= 0x01;
  if (vacuum) motorBits |= 0x02;
  if (mainBrush) motorBits |= 0x04;
  
  sendCommand(OI_MOTORS, motorBits);
  debugPrint("MOTORS command", motorBits);
}

void RoombaOI::setLEDs(uint8_t ledBits, uint8_t powerColor, uint8_t powerIntensity) {
  uint8_t params[3] = {ledBits, powerColor, powerIntensity};
  sendCommand(OI_LEDS, params, 3);
  debugPrint("LEDS command");
}

bool RoombaOI::getSensor(uint8_t sensorId, uint8_t* data, uint8_t dataSize) {
  if (!_connected || !data) return false;
  
  sendCommand(OI_SENSORS, sensorId);
  delay(15); // Wait for response
  
  return readBytes(data, dataSize, 100);
}

uint16_t RoombaOI::getBatteryVoltage() {
  uint8_t data[2];
  if (getSensor(SENSOR_VOLTAGE, data, 2)) {
    return (data[0] << 8) | data[1];
  }
  return 0;
}

int16_t RoombaOI::getBatteryCurrent() {
  uint8_t data[2];
  if (getSensor(SENSOR_CURRENT, data, 2)) {
    return (int16_t)((data[0] << 8) | data[1]);
  }
  return 0;
}

/**
mode : 
0: Off
1: Passive
2: Safe
3: Full
*/
uint8_t RoombaOI::getMode() {
  uint8_t data;
  if (getSensor(SENSOR_MODE, &data, 1)) {
    return data;
  }
  return 0;
}


bool RoombaOI::isWallDetected() {
  uint8_t data;
  if (getSensor(SENSOR_WALL, &data, 1)) {
    return data != 0;
  }
  return false;
}

bool RoombaOI::isBumperPressed() {
  uint8_t data;
  if (getSensor(SENSOR_BUMPS_DROPS, &data, 1)) {
    return (data & 0x03) != 0; // Check bump bits
  }
  return false;
}

bool RoombaOI::startSensorStream(const uint8_t* sensorList, uint8_t numSensors) {
  if (!_connected || !sensorList || numSensors == 0) return false;
  
  sendCommand(OI_STREAM, numSensors);
  
  for (uint8_t i = 0; i < numSensors; i++) {
    _serial->write(sensorList[i]);
  }
  
  debugPrint("Sensor stream started", numSensors);
  return true;
}

bool RoombaOI::stopSensorStream() {
  if (!_connected) return false;
  
  sendCommand(OI_STREAM, 0); // 0 sensors = stop stream
  debugPrint("Sensor stream stopped");
  return true;
}

bool RoombaOI::readStreamData(uint8_t* buffer, uint8_t bufferSize) {
  // Simple stream reading - look for header (19) then size byte
  if (!_connected || !buffer) return false;
  
  // Wait for header byte (19)
  unsigned long timeout = millis() + 100;
  while (millis() < timeout) {
    if (_serial->available() && _serial->read() == 19) {
      // Found header, read size
      uint8_t size = readByte(50);
      if (size > 0 && size <= bufferSize) {
        return readBytes(buffer, size, 100);
      }
    }
  }
  return false;
}

// Private helper methods
void RoombaOI::pulseDD() {
  debugPrint("Pulsing BRC pin");
  for (int i = 0; i < 3; i++) {
    digitalWrite(_brcPin, LOW);
    delay(100);
    digitalWrite(_brcPin, HIGH);
    delay(100);
  }
}

void RoombaOI::sendCommand(uint8_t cmd) {
  if (_connected) {
    _serial->write(cmd);
  }
}

void RoombaOI::sendCommand(uint8_t cmd, uint8_t param) {
  if (_connected) {
    _serial->write(cmd);
    _serial->write(param);
  }
}

void RoombaOI::sendCommand(uint8_t cmd, uint8_t param1, uint8_t param2) {
  if (_connected) {
    _serial->write(cmd);
    _serial->write(param1);
    _serial->write(param2);
  }
}

void RoombaOI::sendCommand(uint8_t cmd, const uint8_t* params, uint8_t numParams) {
  if (_connected && params) {
    _serial->write(cmd);
    for (uint8_t i = 0; i < numParams; i++) {
      _serial->write(params[i]);
    }
  }
}

void RoombaOI::sendInt16(int16_t value) {
  if (_connected) {
    _serial->write((value >> 8) & 0xFF);
    _serial->write(value & 0xFF);
  }
}

uint8_t RoombaOI::readByte(uint16_t timeout) {
  unsigned long start = millis();
  while (!_serial->available() && (millis() - start) < timeout) {
    // Wait for data
  }
  return _serial->available() ? _serial->read() : 0;
}

bool RoombaOI::readBytes(uint8_t* buffer, uint8_t numBytes, uint16_t timeout) {
  if (!buffer) return false;
  
  unsigned long start = millis();
  uint8_t bytesRead = 0;
  
  while (bytesRead < numBytes && (millis() - start) < timeout) {
    if (_serial->available()) {
      buffer[bytesRead++] = _serial->read();
    }
  }
  
  return bytesRead == numBytes;
}

void RoombaOI::debugPrint(const char* msg) {
  if (_debug && msg) {
    Serial.print("RoombaOI: ");
    Serial.println(msg);
  }
}

void RoombaOI::debugPrint(const char* msg, int value) {
  if (_debug && msg) {
    Serial.print("RoombaOI: ");
    Serial.print(msg);
    Serial.print(" = ");
    Serial.println(value);
  }
}

void RoombaOI::debugPrint(const char* msg, uint8_t value) {
  if (_debug && msg) {
    Serial.print("RoombaOI: ");
    Serial.print(msg);
    Serial.print(" = ");
    Serial.println(value);
  }
}

} // namespace ArduRoomba