/* 
  QueryDroneState.ino - Example code demonstrating how to query specific states
  of a DJI Tello drone using command queries with the TelloESP32 library.
  Source code: https://github.com/sagar-koirala/TelloESP32.git
  API reference: https://github.com/sagar-koirala/TelloESP32?tab=readme-ov-file#query-commands
*/

#include "TelloESP32.h"

using namespace TelloControl;

const char *TELLO_SSID = "TELLO-56CC13";  // Replace with your Tello's SSID
const char *TELLO_PASSWORD = "";          // Tello's WiFi password (usually empty)

TelloESP32 tello;  // Create an instance of the TelloESP32 class

// Error handler callback function
void telloErrorHandler(const char* command, const char* errorMessage) {
    Serial.printf("[ERROR] Command '%s': %s\n", command, errorMessage);
}

void setup() {
    Serial.begin(115200);
    Serial.println("TelloESP32 Query Drone State Example");
    
    // Set the error handler callback
    tello.setErrorCallback(telloErrorHandler);

    // Attempt to connect to the Tello drone
    TelloStatus status = tello.connect(TELLO_SSID, TELLO_PASSWORD);
    if (status != TelloStatus::OK) {
        Serial.println("Connection failed!");
        return;
    }
    
    Serial.println("Connected to Tello successfully!");
}

void loop() {
    static unsigned long lastUpdate = 0;
    
    if (tello.isConnected() && (millis() - lastUpdate >= 2000)) {  // Every 2 seconds to avoid overloading with queries
        Serial.println("\n--- Query Results ---");
        
        // Basic Status
        Serial.printf("Battery: %d%%\n", tello.query_battery());
        Serial.printf("Height: %d cm\n", tello.query_height());
        Serial.printf("Flight time: %d s\n", tello.query_time());
        Serial.printf("Speed: %d cm/s\n", tello.query_speed());
        
        // Environmental Data
        Serial.printf("Temperature: %d°C\n", tello.query_temp());
        Serial.printf("Barometer: %.2f m\n", tello.query_baro());
        Serial.printf("TOF Distance: %.1f cm\n", tello.query_tof());
        
        // Connection Info
        Serial.printf("WiFi SNR: %s\n", tello.query_wifi().c_str());
        
        // System Info
        Serial.printf("SDK Version: %s\n", tello.query_sdk().c_str());
        Serial.printf("Serial Number: %s\n", tello.query_sn().c_str());
            
        Serial.println("------------------");
        lastUpdate = millis();
    } else if (!tello.isConnected()) {
        Serial.println("Connection lost!");
        while (true); // Stop execution
    }
}