/**
 * @file ArduRoomba.h
 * @brief Main ArduRoomba library interface
 * 
 * Clean, minimal interface for controlling iRobot Create 2 and compatible Roomba models.
 * Provides convenient methods while maintaining modularity for future extensions.
 */

#ifndef ARDUROOMBA_H
#define ARDUROOMBA_H

#include "RoombaOI.h"

namespace ArduRoomba {

struct SensorData {
    // Timing and status information
    unsigned long nextRefresh;                      //  Time of next scheduled update (ms)
    unsigned long lastSuccessfulRefresh;            //  Time of last successful update (ms)
    uint16_t failedAttempts;                        //  Number of consecutive failed attempts
    
    // Basic sensor data (single byte values)
    uint8_t irOpcode;                               //  Infrared character omni
    uint8_t songNumber;                             //  Current song number
    uint8_t ioStreamNumPackets;                     //  Number of stream packets
    uint8_t mode;                                   //  Current Open Interface mode
    uint8_t chargingState;                          //  Battery charging state
    uint8_t infraredCharacterLeft;                  //  Infrared character left
    uint8_t infraredCharacterRight;                 //  Infrared character right
    int8_t temperature;                             //  Battery temperature (°C)
    
    // Two-byte sensor values
    uint16_t voltage;                               //  Battery voltage (mV)
    int16_t current;                                //  Battery current (mA)
    uint16_t batteryCapacity;                       //  Battery capacity (mAh)
    uint16_t batteryCharge;                         //  Current battery charge (mAh)
    uint8_t dirtDetect;                             //  Dirt detect level (0-255)
    int16_t velocity;                               //  Current velocity (mm/s)
    int16_t rightVelocity;                          //  Right wheel velocity (mm/s)
    int16_t leftVelocity;                           //  Left wheel velocity (mm/s)
    int16_t radius;                                 //  Current turning radius (mm)
    uint16_t leftEncoderCounts;                     //  Left encoder counts
    uint16_t rightEncoderCounts;                    //  Right encoder counts
    int16_t leftMotorCurrent;                       //  Left motor current (mA)
    int16_t rightMotorCurrent;                      //  Right motor current (mA)
    int16_t mainBrushMotorCurrent;                  //  Main brush motor current (mA)
    int16_t sideBrushMotorCurrent;                  //  Side brush motor current (mA)
    
    // Signal strength values
    uint16_t wallSignal;                            //  Wall sensor signal strength
    uint16_t cliffLeftSignal;                       //  Cliff left signal strength
    uint16_t cliffFrontLeftSignal;                  //  Cliff front left signal strength
    uint16_t cliffRightSignal;                      //  Cliff right signal strength
    uint16_t cliffFrontRightSignal;                 //  Cliff front right signal strength
    uint16_t lightBumpLeftSignal;                   //  Light bump left signal strength
    uint16_t lightBumpFrontLeftSignal;              //  Light bump front left signal strength
    uint16_t lightBumpCenterLeftSignal;             //  Light bump center left signal strength
    uint16_t lightBumpCenterRightSignal;            //  Light bump center right signal strength
    uint16_t lightBumpFrontRightSignal;             //  Light bump front right signal strength
    uint16_t lightBumpRightSignal;                  //  Light bump right signal strength
    
    // Boolean sensor flags
    bool wall;                                      //  Wall sensor detected
    bool virtualWall;                               //  Virtual wall detected
    bool cliffLeft;                                 //  Cliff detected on left
    bool cliffFrontLeft;                            //  Cliff detected on front left
    bool cliffRight;                                //  Cliff detected on right
    bool cliffFrontRight;                           //  Cliff detected on front right
    bool songPlaying;                               //  Song currently playing
    
    // Light bumper sensors
    bool lightBumperLeft;                           //  Light bumper left activated
    bool lightBumperFrontLeft;                      //  Light bumper front left activated
    bool lightBumperCenterLeft;                     //  Light bumper center left activated
    bool lightBumperCenterRight;                    //  Light bumper center right activated
    bool lightBumperFrontRight;                     //  Light bumper front right activated
    bool lightBumperRight;                          //  Light bumper right activated
    
    // Charger availability
    bool internalChargerAvailable;                  //  Internal charger available
    bool homeBaseChargerAvailable;                  //  Home base charger available
    
    // Stasis flags
    bool stasisDisabled;                            //  Stasis disabled
    bool stasisToggling;                            //  Stasis toggling
    
    // Button states
    bool cleanButton;                               //  Clean button pressed
    bool spotButton;                                //  Spot button pressed
    bool dockButton;                                //  Dock button pressed
    bool minuteButton;                              //  Minute button pressed
    bool hourButton;                                //  Hour button pressed
    bool dayButton;                                 //  Day button pressed
    bool scheduleButton;                            //  Schedule button pressed (fixed typo)
    bool clockButton;                               //  Clock button pressed
    
    // Wheel and motor overcurrent flags
    bool wheelRightOvercurrent;                     //  Right wheel overcurrent
    bool wheelLeftOvercurrent;                      //  Left wheel overcurrent
    bool mainBrushOvercurrent;                      //  Main brush overcurrent
    bool sideBrushOvercurrent;                      //  Side brush overcurrent
    bool vacuumOvercurrent;                         //  Vacuum overcurrent
    
    // Bump and wheel drop sensors
    bool bumpRight;                                  //  Right bumper activated
    bool bumpLeft;                                   //  Left bumper activated
    bool wheelDropRight;                             //  Right wheel dropped
    bool wheelDropLeft;                              //  Left wheel dropped
    
    SensorData() {
        reset();
    }
    
    /**
     * @brief Resets all sensor data to default values
     */
    void reset() {
        nextRefresh = 0;
        lastSuccessfulRefresh = 0;
        failedAttempts = 0;
        
        irOpcode = 0;
        songNumber = 0;
        ioStreamNumPackets = 0;
        mode = 0;
        chargingState = 0;
        infraredCharacterLeft = 0;
        infraredCharacterRight = 0;
        temperature = 0;
        
        voltage = 0;
        current = 0;
        batteryCapacity = 0;
        batteryCharge = 0;
        dirtDetect = 0;
        velocity = 0;
        rightVelocity = 0;
        leftVelocity = 0;
        radius = 0;
        leftEncoderCounts = 0;
        rightEncoderCounts = 0;
        leftMotorCurrent = 0;
        rightMotorCurrent = 0;
        mainBrushMotorCurrent = 0;
        sideBrushMotorCurrent = 0;
        
        wallSignal = 0;
        cliffLeftSignal = 0;
        cliffFrontLeftSignal = 0;
        cliffRightSignal = 0;
        cliffFrontRightSignal = 0;
        lightBumpLeftSignal = 0;
        lightBumpFrontLeftSignal = 0;
        lightBumpCenterLeftSignal = 0;
        lightBumpCenterRightSignal = 0;
        lightBumpFrontRightSignal = 0;
        lightBumpRightSignal = 0;
        
        wall = false;
        virtualWall = false;
        cliffLeft = false;
        cliffFrontLeft = false;
        cliffRight = false;
        cliffFrontRight = false;
        songPlaying = false;
        
        lightBumperLeft = false;
        lightBumperFrontLeft = false;
        lightBumperCenterLeft = false;
        lightBumperCenterRight = false;
        lightBumperFrontRight = false;
        lightBumperRight = false;
        
        internalChargerAvailable = false;
        homeBaseChargerAvailable = false;
        
        stasisDisabled = false;
        stasisToggling = false;
        
        cleanButton = false;
        spotButton = false;
        dockButton = false;
        minuteButton = false;
        hourButton = false;
        dayButton = false;
        scheduleButton = false;
        clockButton = false;
        
        wheelRightOvercurrent = false;
        wheelLeftOvercurrent = false;
        mainBrushOvercurrent = false;
        sideBrushOvercurrent = false;
        vacuumOvercurrent = false;
        
        bumpRight = false;
        bumpLeft = false;
        wheelDropRight = false;
        wheelDropLeft = false;
    }
    
    bool isFresh(unsigned long maxAge = 1000) const {
        return (millis() - lastSuccessfulRefresh) < maxAge;
    }
    
    /**
     * @brief Gets the age of the sensor data in milliseconds
     * @return Age of data in milliseconds
     */
    unsigned long getAge() const {
        return millis() - lastSuccessfulRefresh;
    }
    

    int getBatteryPercentage() const {
        if (batteryCapacity == 0) return -1;
        return (batteryCharge * 100) / batteryCapacity;
    }
};

class ArduRoomba {
public:
  // Constructor
  ArduRoomba(uint8_t rxPin, uint8_t txPin, uint8_t brcPin);
  
  // Basic lifecycle
  bool begin(uint32_t baudRate = 19200);
  void end();
  bool isConnected() const;
  
  // Simple movement commands
  void moveForward(int16_t speed = 200);
  void moveBackward(int16_t speed = 200);
  void turnLeft(int16_t speed = 200);
  void turnRight(int16_t speed = 200);
  void stop();
  
  // Advanced movement
  void drive(int16_t velocity, int16_t radius);
  void driveDirect(int16_t rightVel, int16_t leftVel);
  
  // Cleaning modes
  void startCleaning();
  void spotClean();
  void dock();
  
  // Basic sensors
  uint16_t getBatteryVoltage();
  int16_t getBatteryCurrent();
  bool isWallDetected();
  bool isBumperPressed();
  
  // Actuators
  void setBrushes(bool main, bool side, bool vacuum = false);
  void setLED(bool debris, bool spot, bool dock, bool checkRobot = false);
  void setPowerLED(uint8_t color, uint8_t intensity = 255);
  
  // Sound
  void beep();
  void playTone(uint8_t note, uint8_t duration);
  
  // Utility
  void setDebug(bool enable);
  
  // Access to underlying OI layer for advanced use
  RoombaOI& getOI() { return _oi; }
  
private:
  RoombaOI _oi;
  bool _debug;
  
  void debugPrint(const char* msg);
  void debugPrint(const char* msg, int value);
};

} // namespace ArduRoomba

#endif