#include "ArduRoomba.h"

ArduRoomba roomba(2, 3, 4); // rxPin, txPin, brcPin

void setup() {
  Serial.begin(19200);
  roomba.roombaSetup();
  roomba.safe();

  // warning don't request to many sensor
  // stream data time slot is only 15ms
  char sensorlist[] = {ARDUROOMBA_SENSOR_MODE,
                       ARDUROOMBA_SENSOR_TEMPERATURE,
                       ARDUROOMBA_SENSOR_VOLTAGE,
                       ARDUROOMBA_SENSOR_BATTERYCHARGE,
                       ARDUROOMBA_SENSOR_BUMPANDWEELSDROPS,
                       ARDUROOMBA_SENSOR_WALL,
                       ARDUROOMBA_SENSOR_CLIFFLEFT,
                       ARDUROOMBA_SENSOR_CLIFFFRONTLEFT,
                       ARDUROOMBA_SENSOR_CLIFFRIGHT,
                       ARDUROOMBA_SENSOR_CLIFFFRONTRIGHT};
  roomba.resetStream();
  roomba.queryStream(sensorlist);
}

ArduRoomba::RoombaInfos infos = {};
void loop() {
  if (roomba.refreshData()) {
    printChanges();
  } else {
    long lastSuccedRefresh = roomba.getLastSuccedRefresh();
    if (millis() - lastSuccedRefresh > 1000) {
      Serial.println("NO SERIAL");
      delay(1000);
    }
  }
}

void printChanges() {
  int newMode = roomba.getMode();
  if (newMode != infos.mode) {
    infos.mode = newMode;
    Serial.print("mode = ");
    Serial.println(infos.mode);
  }

  bool newWall = roomba.getWall();
  if (newWall != infos.wall) {
    infos.wall = newWall;
    Serial.print("wall = ");
    Serial.println(infos.wall);
  }

  bool newCliffLeft = roomba.getCliffLeft();
  if (newCliffLeft != infos.cliffLeft) {
    infos.cliffLeft = newCliffLeft;
    Serial.print("cliffLeft = ");
    Serial.println(infos.cliffLeft);
  }

  bool newCliffFrontLeft = roomba.getCliffFrontLeft();
  if (newCliffFrontLeft != infos.cliffFrontLeft) {
    infos.cliffFrontLeft = newCliffFrontLeft;
    Serial.print("cliffFrontLeft = ");
    Serial.println(infos.cliffFrontLeft);
  }

  bool newCliffRight = roomba.getCliffRight();
  if (newCliffRight != infos.cliffRight) {
    infos.cliffRight = newCliffRight;
    Serial.print("cliffRight = ");
    Serial.println(infos.cliffRight);
  }

  bool newCliffFrontRight = roomba.getCliffFrontRight();
  if (newCliffFrontRight != infos.cliffFrontRight) {
    infos.cliffFrontRight = newCliffFrontRight;
    Serial.print("cliffFrontRight = ");
    Serial.println(infos.cliffFrontRight);
  }

  int newVoltage = roomba.getVoltage();
  if (newVoltage != infos.voltage) {
    infos.voltage = newVoltage;
    Serial.print("voltage = ");
    Serial.println(infos.voltage);
  }

  int newBatteryCharge = roomba.getBatteryCharge();
  if (newBatteryCharge != infos.batteryCharge) {
    infos.batteryCharge = newBatteryCharge;
    Serial.print("batteryCharge = ");
    Serial.println(infos.batteryCharge);
  }

  int newChargingState = roomba.getChargingState();
  if (newChargingState != infos.chargingState) {
    infos.chargingState = newChargingState;
    Serial.print("chargingState = ");
    Serial.println(infos.chargingState);
  }

  int newTemperature = roomba.getTemperature();
  if (newTemperature != infos.temperature) {
    infos.temperature = newTemperature;
    Serial.print("temperature = ");
    Serial.println(infos.temperature);
  }

  bool isBumpRight = roomba.isBumpRight();
  if (infos.bumpRight != isBumpRight) {
    infos.bumpRight = isBumpRight;
    Serial.print("bumpRight = ");
    Serial.println(infos.bumpRight);
  }

  bool isBumpLeft = roomba.isBumpLeft();
  if (infos.bumpLeft != isBumpLeft) {
    infos.bumpLeft = isBumpLeft;
    Serial.print("bumpLeft = ");
    Serial.println(infos.bumpLeft);
  }

  bool isDropWheelRight = roomba.isDropWheelRight();
  if (infos.wheelDropRight != isDropWheelRight) {
    infos.wheelDropRight = isDropWheelRight;
    Serial.print("wheelDropRight = ");
    Serial.println(infos.wheelDropRight);
  }

  bool isDropWheelLeft = roomba.isDropWheelLeft();
  if (infos.wheelDropLeft != isDropWheelLeft) {
    infos.wheelDropLeft = isDropWheelLeft;
    Serial.print("wheelDropLeft = ");
    Serial.println(infos.wheelDropLeft);
  }
}