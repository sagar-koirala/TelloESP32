#include "TelloESP32.h"

using namespace TelloControl;

const char *TELLO_SSID = "TELLO-XXXXXX"; // Replace with your Tello's SSID
const char *TELLO_PASSWORD = "";

TelloESP32 tello;

void setup() {
    Serial.begin(115200);
    Serial.println("TelloESP32 Mission Pads Example");

    if (tello.connect(TELLO_SSID, TELLO_PASSWORD) != TelloStatus::OK) {
        Serial.println("Connection failed!");
        return;
    }

    // Enable mission pad detection
    tello.enable_mission_pads();
    tello.set_mission_pad_detection_direction(0); // Detect in both directions

    // Execute mission pad sequence
    if (tello.takeoff() == TelloStatus::OK) {
        delay(5000);
        
        // Move to different mission pads
        tello.go_xyz_speed(0, 0, 100, 50); // Move up
        delay(3000);
        
        // Add your mission pad navigation sequence here
        
        tello.land();
    }

    // Disable mission pads when done
    tello.disable_mission_pads();
}

void loop() {
    // Empty
}