#include "TelloESP32.h"

using namespace TelloControl;

// Tello drone credentials
const char *TELLO_SSID = "TELLO-56CC13";
const char *TELLO_PASSWORD = "";

TelloESP32 tello;

void setup() {
    Serial.begin(115200);
    Serial.println("TelloESP32 Basic Connection Example");
    
    TelloStatus status = tello.connect(TELLO_SSID, TELLO_PASSWORD);
    if (status != TelloStatus::OK) {
        Serial.println("Failed to connect to Tello");
        return;
    }
    
    Serial.println("Connected to Tello!");
    delay(3000);
    
    // Query battery level
    int battery = tello.query_battery();
    Serial.print("Battery level: ");
    Serial.print(battery);
    Serial.println("%");

    // Query height
    int height = tello.query_height();
    Serial.print("Height: ");
    Serial.print(height);
    Serial.println(" cm");

    // Query speed
    int speed = tello.query_speed();
    Serial.print("Speed: ");
    Serial.print(speed);
    Serial.println(" cm/s");

    // Query flight time
    int flight_time = tello.query_time();
    Serial.print("Flight time: ");
    Serial.print(flight_time);
    Serial.println(" s");

    // Query temperature
    int temperature = tello.query_temp();
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");

    // Query barometer
    float barometer = tello.query_baro();
    Serial.print("Barometer: ");
    Serial.print(barometer);
    Serial.println(" m");

    // Query TOF distance
    float tof = tello.query_tof();
    Serial.print("TOF distance: ");
    Serial.print(tof);
    Serial.println(" cm");

    // Query WiFi SNR
    String wifi = tello.query_wifi();
    Serial.print("WiFi SNR: ");
    Serial.println(wifi);

    // Query SDK version
    String sdk = tello.query_sdk();
    Serial.print("SDK version: ");
    Serial.println(sdk);

    // Query serial number
    String sn = tello.query_sn();
    Serial.print("Serial number: ");
    Serial.println(sn);

    tello.disconnect();
    Serial.println("Tello disconnected!");
}

void loop() {
    // Empty
}