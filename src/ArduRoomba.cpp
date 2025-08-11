/**
 * @file ArduRoomba.cpp
 * @brief Implementation of main ArduRoomba interface
 */

#include "ArduRoomba.h"

namespace ArduRoomba {

ArduRoomba::ArduRoomba(uint8_t rxPin, uint8_t txPin, uint8_t brcPin)
  : _oi(rxPin, txPin, brcPin), _debug(false) {
}

bool ArduRoomba::begin(uint32_t baudRate) {
  debugPrint("Starting ArduRoomba...");
  
  if (_oi.begin(baudRate)) {
    _oi.setDebug(_debug);
    debugPrint("ArduRoomba ready");
    return true;
  }
  
  debugPrint("ArduRoomba failed to start");
  return false;
}

void ArduRoomba::end() {
  _oi.end();
  debugPrint("ArduRoomba stopped");
}

bool ArduRoomba::isConnected() const {
  return _oi.isConnected();
}

// Simple movement commands
void ArduRoomba::moveForward(int16_t speed) {
  debugPrint("Moving forward", speed);
  _oi.drive(speed, DRIVE_STRAIGHT);
}

void ArduRoomba::moveBackward(int16_t speed) {
  debugPrint("Moving backward", speed);
  _oi.drive(-speed, DRIVE_STRAIGHT);
}

void ArduRoomba::turnLeft(int16_t speed) {
  debugPrint("Turning left", speed);
  _oi.drive(speed, DRIVE_TURN_CCW);
}

void ArduRoomba::turnRight(int16_t speed) {
  debugPrint("Turning right", speed);
  _oi.drive(speed, DRIVE_TURN_CW);
}

void ArduRoomba::stop() {
  debugPrint("Stopping");
  _oi.stop();
}

// Advanced movement
void ArduRoomba::drive(int16_t velocity, int16_t radius) {
  _oi.drive(velocity, radius);
}

void ArduRoomba::driveDirect(int16_t rightVel, int16_t leftVel) {
  _oi.driveDirect(rightVel, leftVel);
}

// Cleaning modes
void ArduRoomba::startCleaning() {
  debugPrint("Starting cleaning mode");
  _oi.clean();
}

void ArduRoomba::spotClean() {
  debugPrint("Starting spot cleaning");
  _oi.spot();
}

void ArduRoomba::dock() {
  debugPrint("Seeking dock");
  _oi.seekDock();
}

// Basic sensors
uint16_t ArduRoomba::getBatteryVoltage() {
  return _oi.getBatteryVoltage();
}

int16_t ArduRoomba::getBatteryCurrent() {
  return _oi.getBatteryCurrent();
}

bool ArduRoomba::isWallDetected() {
  return _oi.isWallDetected();
}

bool ArduRoomba::isBumperPressed() {
  return _oi.isBumperPressed();
}

// Actuators
void ArduRoomba::setBrushes(bool main, bool side, bool vacuum) {
  debugPrint("Setting brushes");
  _oi.setMotors(main, side, vacuum);
}

void ArduRoomba::setLED(bool debris, bool spot, bool dock, bool checkRobot) {
  uint8_t ledBits = 0;
  if (debris) ledBits |= 0x01;
  if (spot) ledBits |= 0x02;
  if (dock) ledBits |= 0x04;
  if (checkRobot) ledBits |= 0x08;
  
  _oi.setLEDs(ledBits, 0, 255); // Green power LED
  debugPrint("Setting LEDs", ledBits);
}

void ArduRoomba::setPowerLED(uint8_t color, uint8_t intensity) {
  _oi.setLEDs(0, color, intensity);
  debugPrint("Setting power LED", color);
}

// Sound
void ArduRoomba::beep() {
  playTone(72, 32); // Middle C for half second
}

void ArduRoomba::playTone(uint8_t note, uint8_t duration) {
  // Simple single-note song
  uint8_t songData[4] = {0, 1, note, duration}; // Song 0, 1 note, note, duration
  
  // Define song
  _oi.sendCommand(OI_SONG, songData, 4);
  delay(20);
  
  // Play song
  _oi.sendCommand(OI_PLAY, 0);
  
  debugPrint("Playing tone", note);
}

// Utility
void ArduRoomba::setDebug(bool enable) {
  _debug = enable;
  _oi.setDebug(enable);
  debugPrint("Debug mode", enable);
}

void ArduRoomba::debugPrint(const char* msg) {
  if (_debug && msg) {
    Serial.print("ArduRoomba: ");
    Serial.println(msg);
  }
}

void ArduRoomba::debugPrint(const char* msg, int value) {
  if (_debug && msg) {
    Serial.print("ArduRoomba: ");
    Serial.print(msg);
    Serial.print(" = ");
    Serial.println(value);
  }
}

} // namespace ArduRoomba