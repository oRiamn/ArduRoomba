#include "ArduRoomba.h"

ArduRoomba::ArduRoomba(int rxPin, int txPin, int brcPin)
    : _rxPin(rxPin), _txPin(txPin), _brcPin(brcPin), _irobot(rxPin, txPin), _stateInfos({}), _nextRefresh(0)
{
  // Constructor implementation
}

void ArduRoomba::queryStream() {
  Serial.println("ArduRoomba::queryStream:");
  _irobot.write(148);
  _irobot.write(_nbSensorsStream);
  for (int i = 0; i < _nbSensorsStream; i++) {
      Serial.print(" ");
      Serial.println(_sensorsStream[i], DEC);
     _irobot.write(_sensorsStream[i]);
  }
}

bool ArduRoomba::_readStream() {  
  unsigned long timeout = millis() + ARDUROOMBA_STREAM_TIMEOUT; // stream start every 15ms
  while (!_irobot.available()) {
    if (millis() > timeout) {
      Serial.println("enable to ArduRoomba::_readStream (serial timeout)");
      return false; // Timed out
    }
  }

  int state = ARDUROOMBA_STREAM_WAIT_HEADER;
  _streamBufferSize = 0;
  uint8_t streamSize;
  uint8_t lastChunk;
  uint8_t checksum=0;
  while (_irobot.available()) {
    uint8_t chunk = _irobot.read();
    switch(state) {
      case ARDUROOMBA_STREAM_WAIT_HEADER :
         if(chunk == 19) {
            state = ARDUROOMBA_STREAM_WAIT_SIZE; 
          }
        break;
      case ARDUROOMBA_STREAM_WAIT_SIZE :
          streamSize = chunk;
          state = ARDUROOMBA_STREAM_WAIT_CONTENT; 
      break;
      case ARDUROOMBA_STREAM_WAIT_CONTENT :
          if(_streamBufferSize < streamSize) {
            _streamBuffer[_streamBufferSize] = chunk;
            _streamBufferSize++;
          } else {
            state = ARDUROOMBA_STREAM_WAIT_CHECKSUM;
          }
      break;
      case ARDUROOMBA_STREAM_WAIT_CHECKSUM :
            lastChunk = chunk;
            state = ARDUROOMBA_STREAM_END;
      break;
    }
    checksum += chunk;
  }

  bool isChecksum =  checksum & 0xFF;
  bool isStreamEnd = state == ARDUROOMBA_STREAM_END;
  return isStreamEnd && isChecksum;
}

void ArduRoomba::resetStream() {
  _irobot.write(148);
  int nbS = 0;
  _irobot.write(nbS);
}

bool ArduRoomba::refreshData() {
  long now = millis();
  if(now > _nextRefresh) {
    _nextRefresh = now + ARDUROOMBA_REFRESH_DELAY;
    if(!_readStream() || !_parseStreamBuffer(_streamBuffer, _streamBufferSize, &_stateInfos)) {
      return false;
    }
    _lastSuccedRefresh = now + ARDUROOMBA_REFRESH_DELAY;
    return true;
  }
  return false;
}

long ArduRoomba::getLastSuccedRefresh() {
  return _lastSuccedRefresh;
}

bool ArduRoomba::_parseStreamBuffer(uint8_t* packets, int len, RoombaInfos *infos) {
  int i = 0;
  char packetID;
  while(i < len) {
    packetID = packets[i];
    i++;
    switch(packetID) {
        case ARDUROOMBA_SENSOR_MODE :
          infos->mode = (int) packets[i];
          i++;
          break;
        case ARDUROOMBA_SENSOR_CHARGINGSTATE :
          infos->chargingState = (int) packets[i];
          i++;
          break;
        case ARDUROOMBA_SENSOR_VOLTAGE :
          infos->voltage = (int) (packets[i] * 256 + packets[i+1]);
          i+=2;
          break;
        case ARDUROOMBA_SENSOR_TEMPERATURE :
          infos->temperature = (unsigned int) packets[i];
          i++;
          break;
        case ARDUROOMBA_SENSOR_BATTERYCHARGE :
          infos->batteryCapacity =  (int) (packets[i] * 256 + packets[i]);
          i+=2;
          break;
        case ARDUROOMBA_SENSOR_BATTERYCAPACITY :
          infos->batteryCharge =  (int) (packets[i] * 256 + packets[i]);
          i+=2;
          break;
        case ARDUROOMBA_SENSOR_BUMPANDWEELSDROPS :
          infos->bumpRight = (packets[i] >> 0) & 1;
          infos->bumpLeft = (packets[i] >> 1) & 1;
          infos->wheelDropRight = (packets[i] >> 2) & 1;
          infos->wheelDropLeft = (packets[i] >> 3) & 1;
          i++;
          break;
    default:
          Serial.print("Unhandled Packet ID: ");
          Serial.println(packetID, DEC);
          return false;
          break;
    }
  }
  return true;
}

bool ArduRoomba::isBumpRight() {
  return _stateInfos.bumpRight;
}

bool ArduRoomba::isBumpLeft() {
  return _stateInfos.bumpLeft;
}

bool ArduRoomba::isDropWheelRight() {
  return _stateInfos.wheelDropRight;
}

bool ArduRoomba::isDropWheelLeft() {
  return _stateInfos.wheelDropLeft;
}

int ArduRoomba::getMode() {
  return _stateInfos.mode;
}

int ArduRoomba::getChargingState() {
  return _stateInfos.chargingState;
}

unsigned int ArduRoomba::getTemperature() {
  return _stateInfos.temperature;
}

int ArduRoomba::getBatteryCapacity() {
  return _stateInfos.batteryCapacity;
}

int ArduRoomba::getBatteryCharge() {
  return _stateInfos.batteryCharge;
}

int ArduRoomba::getVoltage() {
  return _stateInfos.voltage;
}



// OI commands
void ArduRoomba::start()
{
  _irobot.write(128);
}

void ArduRoomba::baud(char baudCode)
{
  _irobot.write(129);
  _irobot.write(baudCode);
}

void ArduRoomba::safe()
{
  _irobot.write(131);
}

void ArduRoomba::reset()
{
  _irobot.write(7);
}

void ArduRoomba::full()
{
  _irobot.write(132);
}

void ArduRoomba::clean()
{
  _irobot.write(135);
}

void ArduRoomba::maxClean()
{
  _irobot.write(136);
}

void ArduRoomba::spot()
{
  _irobot.write(134);
}

void ArduRoomba::seekDock()
{
  _irobot.write(143);
}

void ArduRoomba::schedule(ScheduleStore scheduleData)
{
  _irobot.write(167);
  _irobot.write(scheduleData.sunHour);
  _irobot.write(scheduleData.sunMinute);
  _irobot.write(scheduleData.monHour);
  _irobot.write(scheduleData.monMinute);
  _irobot.write(scheduleData.tueHour);
  _irobot.write(scheduleData.tueMinute);
  _irobot.write(scheduleData.wedHour);
  _irobot.write(scheduleData.wedMinute);
  _irobot.write(scheduleData.thuHour);
  _irobot.write(scheduleData.thuMinute);
  _irobot.write(scheduleData.friHour);
  _irobot.write(scheduleData.friMinute);
  _irobot.write(scheduleData.satHour);
  _irobot.write(scheduleData.satMinute);
}

void ArduRoomba::setDayTime(char day, char hour, char minute)
{
  _irobot.write(168);
  _irobot.write(day);
  _irobot.write(hour);
  _irobot.write(minute);
}

void ArduRoomba::power()
{
  _irobot.write(133);
}

// Actuator commands
void ArduRoomba::drive(int velocity, int radius)
{
  _irobot.write(137);
  _irobot.write((velocity >> 8) & 0xFF);
  _irobot.write(velocity & 0xFF);
  _irobot.write((radius >> 8) & 0xFF);
  _irobot.write(radius & 0xFF);
}

void ArduRoomba::driveDirect(int rightVelocity, int leftVelocity)
{
  _irobot.write(145);
  _irobot.write((rightVelocity >> 8) & 0xFF);
  _irobot.write(rightVelocity & 0xFF);
  _irobot.write((leftVelocity >> 8) & 0xFF);
  _irobot.write(leftVelocity & 0xFF);
}

void ArduRoomba::drivePWM(int rightPWM, int leftPWM)
{
  _irobot.write(146);
  _irobot.write((rightPWM >> 8) & 0xFF);
  _irobot.write(rightPWM & 0xFF);
  _irobot.write((leftPWM >> 8) & 0xFF);
  _irobot.write(leftPWM & 0xFF);
}

void ArduRoomba::motors(byte data)
{
  _irobot.write(138);
  _irobot.write(data);
}

void ArduRoomba::pwmMotors(char mainBrushPWM, char sideBrushPWM, char vacuumPWM)
{
  _irobot.write(144);
  _irobot.write(mainBrushPWM);
  _irobot.write(sideBrushPWM);
  _irobot.write(vacuumPWM);
}

void ArduRoomba::leds(int ledBits, int powerColor, int powerIntensity)
{
  _irobot.write(139);
  _irobot.write(ledBits);
  _irobot.write(powerColor);
  _irobot.write(powerIntensity);
}

void ArduRoomba::schedulingLeds(int weekDayLedBits, int scheduleLedBits)
{
  _irobot.write(162);
  _irobot.write(weekDayLedBits);
  _irobot.write(scheduleLedBits);
}

void ArduRoomba::digitLedsRaw(int digitThree, int digitTwo, int digitOne, int digitZero)
{
  _irobot.write(163);
  _irobot.write(digitThree);
  _irobot.write(digitTwo);
  _irobot.write(digitOne);
  _irobot.write(digitZero);
}

void ArduRoomba::song(Song songData)
{
  _irobot.write(140);
  _irobot.write(songData.songNumber);
  _irobot.write(songData.songLength);
  for (int i = 0; i < songData.songLength; i++)
  {
    _irobot.write(songData.notes[i].noteNumber);
    _irobot.write(songData.notes[i].noteDuration);
  }
}

void ArduRoomba::play(int songNumber)
{
  _irobot.write(141);
  _irobot.write(songNumber);
}

// Input commands
void ArduRoomba::sensors(char packetID)
{
  // Read the data and print it to the serial console
  Serial.print("Packet ID: ");
  Serial.print(packetID, DEC);
  Serial.print(", Data: ");

  _irobot.write(142);
  _irobot.write(packetID);

  delay(15);

  // Read the data in chunks
  while (_irobot.available() > 0)
  {
    byte buffer[64];
    size_t len = _irobot.readBytes(buffer, sizeof(buffer));
    for (size_t i = 0; i < len; i++)
    {
      Serial.print(buffer[i], DEC);
      Serial.print(" ");
    }
  }
  Serial.println();
}

bool ArduRoomba::getSerialData(char packetID, uint8_t* destbuffer, int len) {

  // Serial.print("Packet ID: ");
  // Serial.println(packetID, DEC);

  _irobot.write(142);
  _irobot.write(packetID);
  
  while (len-- > 0) {
    unsigned long timeout = millis() + ARDUROOMBA_SERIAL_READ_TIMEOUT;
    while (!_irobot.available()) {
      if (millis() > timeout) {
        Serial.print("enable to ArduRoomba::getSerialData ");
        Serial.print(packetID, DEC);
        Serial.println(" (serial timeout)");
        return false; // Timed out
      }
    }
    *destbuffer++ = _irobot.read();
  }
  return true;
}

bool ArduRoomba::_reqOneByteSensorData(char packetID, RoombaInfos *infos) {
  uint8_t packets[1] = { 0 };
  if(!getSerialData(packetID, packets, 1)) {
    return false;
  }
  uint8_t streambuffer[2] = { 0, 0 };
  streambuffer[0] = packetID;
  streambuffer[1] = packets[0];
  return _parseStreamBuffer(streambuffer, 2, infos);
}

bool ArduRoomba::_reqTwoByteSensorData(char packetID, RoombaInfos *infos) {
  uint8_t packets[2] = { 0, 0 };
  if(getSerialData(packetID, packets, 2)) {
    return false;
  }
  uint8_t streambuffer[3] = { 0, 0, 0 };
  streambuffer[0] = packetID;
  streambuffer[1] = packets[0];
  streambuffer[2] = packets[1];
  return _parseStreamBuffer(streambuffer, 3, infos);
}

bool ArduRoomba::reqMode(RoombaInfos *infos) {
  return _reqOneByteSensorData(ARDUROOMBA_SENSOR_MODE, infos);
}

bool ArduRoomba::reqChargingState(RoombaInfos *infos) {
  return _reqOneByteSensorData(ARDUROOMBA_SENSOR_CHARGINGSTATE, infos);
}

bool ArduRoomba::reqVoltage(RoombaInfos *infos) {
  return _reqOneByteSensorData(ARDUROOMBA_SENSOR_VOLTAGE, infos);
}

bool ArduRoomba::reqTemperature(RoombaInfos *infos) {
  return _reqOneByteSensorData(ARDUROOMBA_SENSOR_TEMPERATURE, infos);
}

bool ArduRoomba::reqBatteryCharge(RoombaInfos *infos) {
  return _reqTwoByteSensorData(ARDUROOMBA_SENSOR_BATTERYCHARGE, infos);
}

bool ArduRoomba::reqBatteryCapacity(RoombaInfos *infos) {
  return _reqTwoByteSensorData(ARDUROOMBA_SENSOR_BATTERYCAPACITY, infos);
}

bool ArduRoomba::reqBumpAndWeelsDrops(RoombaInfos *infos) {
  return _reqOneByteSensorData(ARDUROOMBA_SENSOR_BUMPANDWEELSDROPS, infos);
}

void ArduRoomba::queryList(byte numPackets, byte *packetIDs)
{
  _irobot.write(149);
  _irobot.write(numPackets);
  for (int i = 0; i < numPackets; i++)
  {
    _irobot.write(packetIDs[i]);
  }

  // Read the data and print it to the serial console
  for (int i = 0; i < numPackets; i++)
  {
    Serial.print("Packet ID: ");
    Serial.print(packetIDs[i], DEC);
    Serial.print(", Data: ");
    while (_irobot.available() > 0)
    {
      Serial.print(_irobot.read(), DEC);
      Serial.print(" ");
    }
    Serial.println();
  }
}

// Custom commands
void ArduRoomba::roombaSetup()
{
  pinMode(_brcPin, OUTPUT);
  digitalWrite(_brcPin, HIGH); // Ensure it starts HIGH
  delay(2000);                 // Wait 2 seconds after power on
  for (int i = 0; i < 3; i++)
  {                              // Pulse the BRC pin low three times
    digitalWrite(_brcPin, LOW);  // Bring BRC pin LOW
    delay(100);                  // Wait 100ms
    digitalWrite(_brcPin, HIGH); // Bring BRC pin back HIGH
    delay(100);                  // Wait 100ms before next pulse
  }
  Serial.println("Attempting connection to iRobot OI");
  delay(150);
  _irobot.begin(19200);

  Serial.println("Sending Start to OI");
  delay(150);
  start();

  Serial.println("Sending Safe to OI");
  delay(150);
  safe();

  Serial.println("Connection to iRobot OI SHOULD BE established");
  Serial.println("Verify if CLEAN light has stopped illuminating");
  delay(150);
}

void ArduRoomba::goForward()
{
  _irobot.write(137);   // Opcode for Drive
  _irobot.write(0x01);  // High byte for 500 mm/s
  _irobot.write(0xF4);  // Low byte for 500 mm/s
  _irobot.write(0x80);  // High byte for radius (straight)
  _irobot.write(_zero); // Low byte for radius (straight)
}

void ArduRoomba::goBackward()
{
  _irobot.write(137);   // Opcode for Drive
  _irobot.write(0xFE);  // High byte for -500 mm/s
  _irobot.write(0x0C);  // Low byte for -500 mm/s
  _irobot.write(0x80);  // High byte for radius (straight)
  _irobot.write(_zero); // Low byte for radius (straight)
}

void ArduRoomba::turnLeft()
{
  // Drive command [137], velocity 200 mm/s, radius 1 (turn in place counterclockwise)
  _irobot.write(137);   // Opcode for Drive
  _irobot.write(_zero); // Velocity high byte (200 mm/s)
  _irobot.write(0xC8);  // Velocity low byte (200 mm/s)
  _irobot.write(_zero); // Radius high byte (1)
  _irobot.write(0x01);  // Radius low byte (1)
}

void ArduRoomba::turnRight()
{
  // Drive command [137], velocity 200 mm/s, radius -1 (turn in place clockwise)
  _irobot.write(137);   // Opcode for Drive
  _irobot.write(_zero); // Velocity high byte (200 mm/s)
  _irobot.write(0xC8);  // Velocity low byte (200 mm/s)
  _irobot.write(0xFF);  // Radius high byte (-1)
  _irobot.write(0xFF);  // Radius low byte (-1)
}

void ArduRoomba::halt()
{
  _irobot.write(137);
  _irobot.write(_zero);
  _irobot.write(_zero);
  _irobot.write(_zero);
  _irobot.write(_zero);
}
