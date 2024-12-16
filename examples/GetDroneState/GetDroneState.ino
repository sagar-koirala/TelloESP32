/* 
  GetDroneState.ino - Example code demonstrating how to retrieve and display the real-time 
  state information of a DJI Tello drone using the TelloESP32 library.
  Source code: https://github.com/sagar-koirala/TelloESP32.git
  API reference: https://github.com/sagar-koirala/TelloESP32?tab=readme-ov-file#direct-state-getters
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
    Serial.println("TelloESP32 Get Drone State Example");
    
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
    
    if (tello.isConnected() && (millis() - lastUpdate >= 1000)) {
        Serial.println("\n--- Drone State ---");
        
        // Flight Status
        Serial.printf("Battery: %d%%\n", tello.get_battery());
        Serial.printf("Height: %d cm\n", tello.get_height());
        Serial.printf("Flight time: %d s\n", tello.get_flight_time());
        
        // Environmental Data
        Serial.printf("Temperature: %.1f°C\n", tello.get_temperature());
        Serial.printf("Barometer: %.2f m\n", tello.get_barometer());
        Serial.printf("TOF Distance: %.1f cm\n", tello.get_tof());
        
        // Orientation
        Serial.printf("Pitch: %d°\n", tello.get_pitch());
        Serial.printf("Roll: %d°\n", tello.get_roll());
        Serial.printf("Yaw: %d°\n", tello.get_yaw());
        
        // Motion Data
        Serial.printf("Speed - X: %d, Y: %d, Z: %d cm/s\n", 
            tello.get_speed_x(), tello.get_speed_y(), tello.get_speed_z());
        Serial.printf("Acceleration - X: %.2f, Y: %.2f, Z: %.2f cm/s²\n", 
            tello.get_acceleration_x(), tello.get_acceleration_y(), tello.get_acceleration_z());
            
        Serial.println("------------------");
        lastUpdate = millis();
    } else if (!tello.isConnected()) {
        Serial.println("Connection lost!");
        while (true); // Stop execution
    }
}