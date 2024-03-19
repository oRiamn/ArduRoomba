#ifndef ArduRoomba_h
#define ArduRoomba_h

#include <Arduino.h>
#include <SoftwareSerial.h>

#define ARDUROOMBA_LOGGING 1
#if ARDUROOMBA_LOGGING
#define ARDUROOMBA_LOG(...) Serial.print(__VA_ARGS__)
#else
#define ARDUROOMBA_LOG(msg, ...)
#endif

#define ARDUROOMBA_DEBUGING 0
#if ARDUROOMBA_DEBUGING
#define ARDUROOMBA_DEBUG(...) Serial.print(__VA_ARGS__)
#else
#define ARDUROOMBA_DEBUG(msg, ...)
#endif

#define ARDUROOMBA_ERROR_LOG 1
#if ARDUROOMBA_ERROR_LOG
#define ARDUROOMBA_ERROR(...) Serial.print(__VA_ARGS__)
#else
#define ARDUROOMBA_ERROR(msg, ...)
#endif

#define ARDUROOMBA_SERIAL_READ_TIMEOUT 100

#define ARDUROOMBA_REFRESH_DELAY 90
#define ARDUROOMBA_STREAM_TIMEOUT 30

#define ARDUROOMBA_STREAM_WAIT_HEADER 0
#define ARDUROOMBA_STREAM_WAIT_SIZE 1
#define ARDUROOMBA_STREAM_WAIT_CONTENT 2
#define ARDUROOMBA_STREAM_WAIT_CHECKSUM 3
#define ARDUROOMBA_STREAM_END 4;

#define ARDUROOMBA_SENSOR_MODE 35
#define ARDUROOMBA_SENSOR_CHARGINGSTATE 21
#define ARDUROOMBA_SENSOR_VOLTAGE 22
#define ARDUROOMBA_SENSOR_TEMPERATURE 24
#define ARDUROOMBA_SENSOR_BATTERYCHARGE 25
#define ARDUROOMBA_SENSOR_BATTERYCAPACITY 26
#define ARDUROOMBA_SENSOR_BUMPANDWEELSDROPS 7
#define ARDUROOMBA_SENSOR_WALL 8
#define ARDUROOMBA_SENSOR_CLIFFLEFT 9
#define ARDUROOMBA_SENSOR_CLIFFFRONTLEFT 10
#define ARDUROOMBA_SENSOR_CLIFFRIGHT 12
#define ARDUROOMBA_SENSOR_CLIFFFRONTRIGHT 11
#define ARDUROOMBA_SENSOR_WHEELOVERCURRENTS 14
#define ARDUROOMBA_SENSOR_DIRTDETECT 15
#define ARDUROOMBA_SENSOR_VIRTUALWALL 13
#define ARDUROOMBA_SENSOR_IROPCODE 17
#define ARDUROOMBA_SENSOR_CHARGERAVAILABLE 34

class ArduRoomba {
public:
  // Constructor
  ArduRoomba(int rxPin, int txPin, int brcPin); // Constructor

  // Custom structs to use in main code.
  struct Note {
    byte noteNumber;   // MIDI note number (31 - 127)
    byte noteDuration; // Duration of the note in 1/64th of a second
  };

  struct Song {
    byte songNumber; // Song number (0 - 4)
    byte songLength; // Number of notes in the song (1 - 16)
    Note notes[16];  // Array of notes, up to 16
  };

  struct RoombaInfos {
    int mode;
    int chargingState;
    int voltage;
    unsigned int temperature;
    int batteryCapacity;
    int batteryCharge;
    int dirtdetect;
    int irOpcode;
    int chargerAvailable;

    bool wall;
    bool virtualWall;
    bool cliffLeft;
    bool cliffFrontLeft;
    bool cliffRight;
    bool cliffFrontRight;

    bool wheelRightOvercurrent;
    bool wheelLeftOvercurrent;
    bool mainBrushOvercurrent;
    bool sideBrushOvercurrent;
    bool vacuumOvercurrent; // no wacuum for serie 600

    bool bumpRight;      // Bump Right ?
    bool bumpLeft;       // Bump Left?
    bool wheelDropRight; // Wheel Drop Right ?
    bool wheelDropLeft;  // Wheel Drop Left ?
  };

  struct ScheduleStore {
    byte days;
    byte sunHour;
    byte sunMinute;
    byte monHour;
    byte monMinute;
    byte tueHour;
    byte tueMinute;
    byte wedHour;
    byte wedMinute;
    byte thuHour;
    byte thuMinute;
    byte friHour;
    byte friMinute;
    byte satHour;
    byte satMinute;
  };

  // OI commands
  void start();             // Start the OI
  void baud(char baudCode); // Set the baud rate
  void safe();              // Put the OI into Safe mode
  void reset();    // Send "reset" command (equivalent to battery replacment)
  void full();     // Put the OI into Full mode
  void clean();    // Start the cleaning mode
  void maxClean(); // Start the maximum time cleaning mode
  void spot();     // Start the spot cleaning mode
  void seekDock(); // Send the robot to the dock
  void schedule(ScheduleStore scheduleData);         // Set the schedule
  void setDayTime(char day, char hour, char minute); // Set the day and time
  void power();                                      // Power down the OI

  // Actuator commands
  void drive(int velocity, int radius); // Drive the robot
  void driveDirect(int rightVelocity,
                   int leftVelocity);       // Drive the robot directly
  void drivePWM(int rightPWM, int leftPWM); // Drive the robot with PWM
  void motors(byte data);                   // Control the motors
  void pwmMotors(char mainBrushPWM, char sideBrushPWM,
                 char vacuumPWM); // Control the PWM of the motors
  void leds(int ledBits, int powerColor,
            int powerIntensity); // Control the LEDs
  void schedulingLeds(int weekDayLedBits,
                      int scheduleLedBits); // Control the scheduling LEDs
  void digitLedsRaw(int digitThree, int digitTwo, int digitOne,
                    int digitZero); // Control the digit LEDs
  void song(Song songData);         // Load a song
  void play(int songNumber);        // Play a song

  // Input commands
  void sensors(char packetID); // Request a sensor packet
  void queryList(byte numPackets,
                 byte *packetIDs); // Request a list of sensor packets
  bool getSerialData(
      char packetID, uint8_t *destbuffer,
      int len); // Request a sensor packet and fill buffer with response

  void queryStream();
  void resetStream();
  bool refreshData();

  // state getters
  long getLastSuccedRefresh();
  bool getWall();
  bool getVirtualWall();
  bool getCliffLeft();
  bool getCliffFrontLeft();
  bool getCliffRight();
  bool getCliffFrontRight();
  bool isBumpRight();
  bool isBumpLeft();
  bool isDropWheelRight();
  bool isDropWheelLeft();
  int getMode();
  int getChargerAvailable();
  int getIrOpcode();
  int getDirtDetect();
  int getChargingState();
  unsigned int getTemperature();
  int getBatteryCapacity();
  int getBatteryCharge();
  int getVoltage();
  bool isWheelRightOvercurrent();
  bool isWheelLeftOvercurrent();
  bool isMainBrushOvercurrent();
  bool isSideBrushOvercurrent();
  bool isVacuumOvercurrent();

  // sensor request
  bool reqMode(RoombaInfos *infos);
  bool reqChargerAvailable(RoombaInfos *infos);
  bool reqIrOpcode(RoombaInfos *infos);
  bool reqDirtDetect(RoombaInfos *infos);
  bool reqChargingState(RoombaInfos *infos);
  bool reqVoltage(RoombaInfos *infos);
  bool reqTemperature(RoombaInfos *infos);
  bool reqBatteryCharge(RoombaInfos *infos);
  bool reqBatteryCapacity(RoombaInfos *infos);
  bool reqWall(RoombaInfos *infos);
  bool reqVirtualWall(RoombaInfos *infos);
  bool reqCliffLeft(RoombaInfos *infos);
  bool reqCliffFrontLeft(RoombaInfos *infos);
  bool reqCliffRight(RoombaInfos *infos);
  bool reqCliffFrontRight(RoombaInfos *infos);
  bool reqBumpAndWeelsDrops(RoombaInfos *infos);
  bool reqWeelsOvercurrents(RoombaInfos *infos);

  // Custom commands
  void roombaSetup(); // Setup the Roomba
  void goForward();   // Move the Roomba forward
  void goBackward();  // Move the Roomba backward
  void turnLeft();    // Turn the Roomba left
  void turnRight();   // Turn the Roomba right
  void halt();        // Stop the Roomba

private:
  const byte _zero = 0x00;
  int _rxPin, _txPin, _brcPin;
  SoftwareSerial
      _irobot; // SoftwareSerial instance for communication with the Roomba

  long _nextRefresh;
  long _lastSuccedRefresh;
  RoombaInfos _stateInfos;

  uint8_t _streamBuffer[100] = {};
  int _nbSensorsStream = 17;
  int _streamBufferSize = 0;
  char _sensorsStream[20] = {ARDUROOMBA_SENSOR_MODE,
                             ARDUROOMBA_SENSOR_IROPCODE,
                             ARDUROOMBA_SENSOR_CHARGINGSTATE,
                             ARDUROOMBA_SENSOR_TEMPERATURE,
                             ARDUROOMBA_SENSOR_VOLTAGE,
                             ARDUROOMBA_SENSOR_BATTERYCHARGE,
                             ARDUROOMBA_SENSOR_CHARGERAVAILABLE,
                             ARDUROOMBA_SENSOR_BATTERYCAPACITY,
                             ARDUROOMBA_SENSOR_BUMPANDWEELSDROPS,
                             ARDUROOMBA_SENSOR_VIRTUALWALL,
                             ARDUROOMBA_SENSOR_WALL,
                             ARDUROOMBA_SENSOR_CLIFFLEFT,
                             ARDUROOMBA_SENSOR_CLIFFFRONTLEFT,
                             ARDUROOMBA_SENSOR_CLIFFRIGHT,
                             ARDUROOMBA_SENSOR_CLIFFFRONTRIGHT,
                             ARDUROOMBA_SENSOR_WHEELOVERCURRENTS,
                             ARDUROOMBA_SENSOR_DIRTDETECT};

  bool _reqNByteSensorData(char packetID, int len, RoombaInfos *infos);
  bool _readStream();
  bool _parseStreamBuffer(uint8_t *packets, int len, RoombaInfos *infos);
  uint8_t _parseOneByteStreamBuffer(uint8_t *packets, int &start);
  int _parseTwoByteStreamBuffer(uint8_t *packets, int &start);
};

#endif
