/**
 * WiringTest.ino
 * 
 * Simple example for checking the wiring
 *
 */

#include "ArduRoomba.h"

// Pin connections to Roomba
// RX Pin (Roomba TX) -> Pin 2  
// TX Pin (Roomba RX) -> Pin 3
// BRC Pin (Roomba DD) -> Pin 4
ArduRoomba::ArduRoomba roomba(2, 3, 4);

void setup() {
  Serial.begin(19200);
  // Enable debug output
  roomba.setDebug(true);
  // Initialize connection to Roomba
  if (roomba.begin(19200)) {
    Serial.println("Roomba connected successfully!");
     // Beep to indicate completion
    for (int i=0;i<3;i++) {
        Serial.println("Beeep!");
        roomba.beep();
        delay(1000);
    }
  } else {
    Serial.println("Failed to connect to Roomba!");
    Serial.println("Check wiring and power.");
  }
}

void loop() {
  // Nothing to do in main loop for this example
}