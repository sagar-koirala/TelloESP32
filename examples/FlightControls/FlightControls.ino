#include "TelloESP32.h"

using namespace TelloControl;

const char *TELLO_SSID = "TELLO-XXXXXX"; // Replace with your Tello's SSID
const char *TELLO_PASSWORD = "";

TelloESP32 tello;

void setup() {
    Serial.begin(115200);
    Serial.println("TelloESP32 Flight Controls Example");
    Serial.println("Commands: t(takeoff), l(land), w(up), s(down),");
    Serial.println("         a(left), d(right), i(forward), k(back),");
    Serial.println("         j(rotate left), l(rotate right), q(quit)");

    if (tello.connect(TELLO_SSID, TELLO_PASSWORD) != TelloStatus::OK) {
        Serial.println("Connection failed!");
        return;
    }
}

void loop() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        
        switch (cmd) {
            case 't': tello.takeoff(); break;
            case 'l': tello.land(); break;
            case 'w': tello.move_up(30); break;
            case 's': tello.move_down(30); break;
            case 'a': tello.move_left(30); break;
            case 'd': tello.move_right(30); break;
            case 'i': tello.move_forward(30); break;
            case 'k': tello.move_back(30); break;
            case 'j': tello.rotate_counter_clockwise(30); break;
            case 'l': tello.rotate_clockwise(30); break;
            case 'q': tello.land(); delay(5000); tello.disconnect(); break;
        }
    }

    // Print drone state every second
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate >= 1000) {
        Serial.printf("Battery: %d%%, Height: %dcm\n", 
                     tello.get_battery(), tello.get_height());
        lastUpdate = millis();
    }
}