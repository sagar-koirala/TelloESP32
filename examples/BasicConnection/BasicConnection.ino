#include "TelloESP32.h"

using namespace TelloControl;

// Tello drone credentials
const char *TELLO_SSID = "TELLO-56CC13";
const char *TELLO_PASSWORD = "";

TelloESP32 tello;

void setup() {
    Serial.begin(115200);
    Serial.println("TelloESP32 Basic Connection Example");
    
    Serial.println("Connecting to Tello");
    TelloStatus status = tello.connect(TELLO_SSID, TELLO_PASSWORD);
    if (status != TelloStatus::OK) {
        Serial.println("Failed to connect to Tello");
        return;
    }
    
    Serial.println("Connectioin successful!");
    delay(3000);
    
    Serial.println("Disconnecting Tello");
    tello.disconnect();
    Serial.println("Tello disconnected!");
}

void loop() {
    // Empty
}
