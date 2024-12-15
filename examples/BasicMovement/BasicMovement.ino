/* 
  BasicMovement.ino - Example demonstrating basic flight movements with a DJI Tello drone
  using the TelloESP32 library. This program performs a simple flight sequence while
  demonstrating proper error handling and command verification.
  Source code: https://github.com/sagar-koirala/TelloESP32.git
  API reference: https://github.com/sagar-koirala/TelloESP32?tab=readme-ov-file#movement-commands
*/

#include "TelloESP32.h"

using namespace TelloControl;

const char* TELLO_SSID = "TELLO-56CC13";  // Replace with your Tello's SSID
const char* TELLO_PASSWORD = "";          // Tello's WiFi password (usually empty)

TelloESP32 tello;

// Error handler callback function
void onTelloError(const char* command, const char* errorMessage) {
    Serial.printf("Error during '%s': %s\n", command, errorMessage);
}

void setup() {
    Serial.begin(115200);
    Serial.println("TelloESP32 Basic Movement Example");

    tello.setErrorCallback(onTelloError);
    
    // Connect to Tello
    tello.connect(TELLO_SSID, TELLO_PASSWORD);
    Serial.println("Connected! Starting flight sequence...");

    // Basic flight sequence
    Serial.println("1. Taking off...");
    tello.takeoff();
    delay(2000);

    Serial.println("2. Moving in square pattern...");
    tello.move_forward(30);
    delay(2000);
    tello.move_right(30);
    delay(2000);
    tello.move_back(30);
    delay(2000);
    tello.move_left(30);
    delay(2000);

    Serial.println("3. Rotating 360 degrees...");
    tello.rotate_clockwise(360);
    delay(2000);

    Serial.println("4. Landing...");
    tello.land();
    delay(2000);

    tello.disconnect();
    Serial.println("Flight sequence completed!");
}

void loop() {
    // All operations are in setup()
}