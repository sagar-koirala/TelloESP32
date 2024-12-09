#include "TelloESP32.h"

using namespace TelloControl;

const char *TELLO_SSID = "TELLO-56CC13";
const char *TELLO_PASSWORD = "";

TelloESP32 tello;

void setup() {
    Serial.begin(115200);
    Serial.println("TelloESP32 Get State Example with Raw Data");

    TelloStatus status = tello.connect(TELLO_SSID, TELLO_PASSWORD);
    if (status != TelloStatus::OK) {
        Serial.println("Failed to connect to Tello");
        return;
    }

    Serial.println("Connected to Tello!");
}

void loop() {
    if (tello.isConnected()) {
        Serial.println("\n--- Drone State ---");
        Serial.printf("Height: %d cm\n", tello.get_height());
        Serial.printf("Battery: %d%%\n", tello.get_battery());
        Serial.printf("Flight time: %d s\n", tello.get_flight_time());
        Serial.printf("Temperature: %.1f °C\n", tello.get_temperature());
        Serial.printf("Barometer: %.2f m\n", tello.get_barometer());
        Serial.printf("TOF: %.1f cm\n", tello.get_tof());
        Serial.printf("Attitude - Pitch: %d°, Roll: %d°, Yaw: %d°\n", 
                    tello.get_pitch(), tello.get_roll(), tello.get_yaw());
        Serial.printf("Speed - X: %d, Y: %d, Z: %d cm/s\n", 
                    tello.get_speed_x(), tello.get_speed_y(), tello.get_speed_z());
        Serial.printf("Acceleration - X: %.2f, Y: %.2f, Z: %.2f cm/s²\n", 
                    tello.get_acceleration_x(), tello.get_acceleration_y(), tello.get_acceleration_z());
        Serial.println("-----------------\n");
        delay(100);
    }
    else {
        Serial.println("Tello not connected!");
        while(1);
    }
}