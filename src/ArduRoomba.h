#ifndef ArduRoomba_h
#define ArduRoomba_h

#include <Arduino.h>
#include <SoftwareSerial.h>

#define ARDUROOMBA_SERIAL_READ_TIMEOUT 100
#define ARDUROOMBA_REFRESH_DELAY 70
#define ARDUROOMBA_STREAM_TIMEOUT 20

#define ARDUROOMBA_SENSOR_MODE 35
#define ARDUROOMBA_SENSOR_CHARGINGSTATE 21
#define ARDUROOMBA_SENSOR_VOLTAGE 22
#define ARDUROOMBA_SENSOR_TEMPERATURE 24
#define ARDUROOMBA_SENSOR_BATTERYCHARGE 25
#define ARDUROOMBA_SENSOR_BATTERYCAPACITY 26
#define ARDUROOMBA_SENSOR_BUMPANDWEELSDROPS 7

#define ARDUROOMBA_STREAM_WAIT_HEADER 0
#define ARDUROOMBA_STREAM_WAIT_SIZE 1
#define ARDUROOMBA_STREAM_WAIT_CONTENT 2
#define ARDUROOMBA_STREAM_WAIT_CHECKSUM 3
#define ARDUROOMBA_STREAM_END 4;


class ArduRoomba
{
public:
  // Constructor
  ArduRoomba(int rxPin, int txPin, int brcPin); // Constructor

  // Custom structs to use in main code.
  struct Note
  {
    byte noteNumber;   // MIDI note number (31 - 127)
    byte noteDuration; // Duration of the note in 1/64th of a second
  };

  struct Song
  {
    byte songNumber; // Song number (0 - 4)
    byte songLength; // Number of notes in the song (1 - 16)
    Note notes[16];  // Array of notes, up to 16
  };

  struct BumpAndWeelsDrops
  {
    bool bumpRight; // Bump Right ?
    bool bumpLeft; // Bump Left?
    bool wheelRight; // Wheel Drop Right ?
    bool wheelLeft; // Wheel Drop Left ?
  };

  struct RoombaInfos
  {
    int mode;
    int chargingState;
    int voltage;
    unsigned int temperature;
    int batteryCapacity;
    int batteryCharge;
  };

  

  struct ScheduleStore
  {
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
  void start();                                      // Start the OI
  void baud(char baudCode);                          // Set the baud rate
  void safe();                                       // Put the OI into Safe mode
  void reset();                                      // Send "reset" command (equivalent to battery replacment)
  void full();                                       // Put the OI into Full mode
  void clean();                                      // Start the cleaning mode
  void maxClean();                                        // Start the maximum time cleaning mode
  void spot();                                       // Start the spot cleaning mode
  void seekDock();                                   // Send the robot to the dock
  void schedule(ScheduleStore scheduleData);         // Set the schedule
  void setDayTime(char day, char hour, char minute); // Set the day and time
  void power();                                      // Power down the OI

  // Actuator commands
  void drive(int velocity, int radius);                                         // Drive the robot
  void driveDirect(int rightVelocity, int leftVelocity);                        // Drive the robot directly
  void drivePWM(int rightPWM, int leftPWM);                                     // Drive the robot with PWM
  void motors(byte data);                                                       // Control the motors
  void pwmMotors(char mainBrushPWM, char sideBrushPWM, char vacuumPWM);         // Control the PWM of the motors
  void leds(int ledBits, int powerColor, int powerIntensity);                   // Control the LEDs
  void schedulingLeds(int weekDayLedBits, int scheduleLedBits);                 // Control the scheduling LEDs
  void digitLedsRaw(int digitThree, int digitTwo, int digitOne, int digitZero); // Control the digit LEDs
  void song(Song songData);                                                     // Load a song
  void play(int songNumber);                                                    // Play a song

  // Input commands
  void sensors(char packetID);                      // Request a sensor packet
  void queryList(byte numPackets, byte *packetIDs); // Request a list of sensor packets
  bool getSerialData(char packetID, uint8_t* destbuffer, int len); // Request a sensor packet and fill buffer with response
  
  bool queryStream();
  bool resetStream();
  bool refreshData();
  long getLastSuccedRefresh();

  bool isBumpRight();
  bool isBumpLeft();
  bool isDropWheelRight();
  bool isDropWheelLeft();

  int getMode();                                        // Request sensor packet "mode"
  int getChargingState();                               // Request sensor packet "charging state"                  // Request sensor packet "voltage" (voltage of Roomba's battery in milivolts)
  unsigned int getTemperature();                       // Request sensor packet "temperature" (temperature of Roomba's battery in degrees Celsius)
  int getBatteryCapacity();
  int getBatteryCharge();
  int getVoltage();

  // sensor request
  int reqMode();                                        // Request sensor packet "mode"
  int reqChargingState();                               // Request sensor packet "charging state"
  int reqVoltage();                                     // Request sensor packet "voltage" (voltage of Roomba's battery in milivolts)
  unsigned int reqTemperature();                        // Request sensor packet "temperature" (temperature of Roomba's battery in degrees Celsius)
  int reqBatteryCharge();                               // Request sensor packet "battery charge" (the current charge of Roomba's battery in miliamp-hours)
  int reqBatteryCapacity();                             // Request sensor packet "battery capacity" (the estimated charge capacity of Roomba's battery in miliamp-hours)
  bool reqBumpAndWeelsDrops(BumpAndWeelsDrops *drops);  // Request sensor packet "Bumps and Wheel Drops" (the state of the bumper and wheel drop sensor)
  
  
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
  SoftwareSerial _irobot; // SoftwareSerial instance for communication with the Roomba
  
  long _nextRefresh;
  long _lastSuccedRefresh;
  BumpAndWeelsDrops _drops;
  RoombaInfos _stateInfos;

  uint8_t _streamBuffer[100] = {};
  int _nbSensorsStream = 7;
  int _streamBufferSize = 0;
  char _sensorsStream[10] = {
    ARDUROOMBA_SENSOR_MODE,
    ARDUROOMBA_SENSOR_CHARGINGSTATE,
    ARDUROOMBA_SENSOR_TEMPERATURE,
    ARDUROOMBA_SENSOR_VOLTAGE,
    ARDUROOMBA_SENSOR_BATTERYCHARGE,
    ARDUROOMBA_SENSOR_BATTERYCAPACITY,
    ARDUROOMBA_SENSOR_BUMPANDWEELSDROPS
  };
  
  int _readOneByteSensorData(char packetID);
  int _readTwoByteSensorData(char packetID);

  bool _readStream();
  bool _parseStreamBuffer(uint8_t* packets, int len);
};

#endif
