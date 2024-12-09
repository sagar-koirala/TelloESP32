#include "TelloESP32.h"

using namespace TelloControl;

const char *TELLO_SSID = "TELLO-56CC13"; // Replace with your Tello's SSID
const char *TELLO_PASSWORD = "";

TelloESP32 tello;

void setup()
{
    Serial.begin(115200);
    Serial.println("TelloESP32 Basic Movement Example");

    if (tello.connect(TELLO_SSID, TELLO_PASSWORD) != TelloStatus::OK)
    {
        Serial.println("Connection failed!");
        return;
    }

    Serial.println("Executing movement sequence...");

    // Take off
    Serial.print("Take off....");
    if (tello.takeoff() == TelloStatus::OK)
    {
      Serial.println("OK");
        delay(1000); // Wait for stable hover

        // Move in a square pattern
        tello.move_forward(20);
        delay(1000);
        tello.move_right(20);
        delay(1000);
        tello.move_back(20);
        delay(1000);
        tello.move_left(20);
        delay(1000);

        // Rotate 360 degrees
        tello.rotate_clockwise(360);
        delay(1000);

        // Land
        Serial.print("Land....");
        tello.land();
        Serial.print("OK");
    }
}

void loop()
{
    // Empty
}