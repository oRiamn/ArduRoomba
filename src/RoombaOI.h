/**
 * @file RoombaOI.h
 * @brief iRobot Open Interface communication layer
 * 
 * Minimal, focused implementation of the iRobot Open Interface protocol.
 * Handles low-level serial communication, command sending, and data reception.
 */

#ifndef ROOMBAOI_H
#define ROOMBAOI_H

#include <Arduino.h>
#include <SoftwareSerial.h>

namespace ArduRoomba {

// OI Command opcodes
#define OI_START        128
#define OI_BAUD         129
#define OI_SAFE         131
#define OI_FULL         132
#define OI_POWER        133
#define OI_SPOT         134
#define OI_CLEAN        135
#define OI_MAX_CLEAN    136
#define OI_DRIVE        137
#define OI_MOTORS       138
#define OI_LEDS         139
#define OI_SONG         140
#define OI_PLAY         141
#define OI_SENSORS      142
#define OI_SEEK_DOCK    143
#define OI_DRIVE_DIRECT 145
#define OI_STREAM       148

// Common sensor packet IDs
#define SENSOR_BUMPS_DROPS      7
#define SENSOR_WALL             8
#define SENSOR_CLIFF_LEFT       9
#define SENSOR_CLIFF_FRONT_LEFT 10
#define SENSOR_CLIFF_FRONT_RIGHT 11
#define SENSOR_CLIFF_RIGHT      12
#define SENSOR_VIRTUAL_WALL     13
#define SENSOR_BUTTONS          18
#define SENSOR_DISTANCE         19
#define SENSOR_ANGLE           20
#define SENSOR_CHARGING_STATE   21
#define SENSOR_VOLTAGE         22
#define SENSOR_CURRENT         23
#define SENSOR_TEMPERATURE     24
#define SENSOR_BATTERY_CHARGE  25
#define SENSOR_BATTERY_CAPACITY 26

// Drive constants
#define DRIVE_STRAIGHT     32768
#define DRIVE_TURN_CCW     1
#define DRIVE_TURN_CW      -1
#define MAX_VELOCITY       500
#define MIN_VELOCITY       -500

class RoombaOI {
public:
  RoombaOI(uint8_t rxPin, uint8_t txPin, uint8_t brcPin);
  
  // Basic setup
  bool begin(uint32_t baudRate = 19200);
  void end();
  bool isConnected() const { return _connected; }
  
  // Core OI commands
  void start();
  void safeMode();
  void fullMode();
  void powerOff();
  
  // Movement
  void drive(int16_t velocity, int16_t radius);
  void driveDirect(int16_t rightVel, int16_t leftVel);
  void stop() { drive(0, 0); }
  
  // Cleaning modes
  void clean();
  void spot();
  void seekDock();
  
  // Actuators
  void setMotors(bool mainBrush, bool sideBrush, bool vacuum);
  void setLEDs(uint8_t ledBits, uint8_t powerColor, uint8_t powerIntensity);
  
  // Sensors
  bool getSensor(uint8_t sensorId, uint8_t* data, uint8_t dataSize);
  uint16_t getBatteryVoltage();
  int16_t getBatteryCurrent();
  bool isWallDetected();
  bool isBumperPressed();
  
  // Streaming (basic)
  bool startSensorStream(const uint8_t* sensorList, uint8_t numSensors);
  bool stopSensorStream();
  bool readStreamData(uint8_t* buffer, uint8_t bufferSize);
  
  // Debug
  void setDebug(bool enable) { _debug = enable; }


  void sendCommand(uint8_t cmd);
  void sendCommand(uint8_t cmd, uint8_t param);
  void sendCommand(uint8_t cmd, uint8_t param1, uint8_t param2);
  void sendCommand(uint8_t cmd, const uint8_t* params, uint8_t numParams);
  
private:
  SoftwareSerial* _serial;
  uint8_t _rxPin, _txPin, _brcPin;
  bool _connected;
  bool _debug;
  
  // Internal helpers
  void pulseDD();

  void sendInt16(int16_t value);
  
  uint8_t readByte(uint16_t timeout = 100);
  bool readBytes(uint8_t* buffer, uint8_t numBytes, uint16_t timeout = 100);
  
  void debugPrint(const char* msg);
  void debugPrint(const char* msg, int value);
};

} // namespace ArduRoomba

#endif