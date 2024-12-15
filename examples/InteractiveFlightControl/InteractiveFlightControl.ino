/* 
  InteractiveFlightControl.ino - Example code demonstrating interactive flight control of a DJI Tello drone 
  using the TelloESP32 library. This program connects to the drone and allows the user to control 
  the drone's movements via serial input commands. It includes error handling and real-time status updates.
  Source code: https://github.com/sagar-koirala/TelloESP32.git
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

void printCommands() {
    Serial.println("\nAvailable Commands:");
    Serial.println("t - takeoff");
    Serial.println("l - land");
    Serial.println("w - up 30cm");
    Serial.println("s - down 30cm");
    Serial.println("a - left 30cm");
    Serial.println("d - right 30cm");
    Serial.println("i - forward 30cm");
    Serial.println("k - back 30cm");
    Serial.println("j - rotate left 30°");
    Serial.println("r - rotate right 30°");
    Serial.println("q - quit program");
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    Serial.println("TelloESP32 Interactive Flight Control Example");
    
    // Set the error handler callback
    tello.setErrorCallback(telloErrorHandler);

    // Attempt to connect to the Tello drone
    TelloStatus status = tello.connect(TELLO_SSID, TELLO_PASSWORD);
    if (status != TelloStatus::OK) {
        Serial.println("Connection failed!");
        return;
    }
    
    Serial.println("Connected to Tello successfully!");
    printCommands();
}

void loop() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        TelloStatus status;

        // Flush any remaining characters
        while(Serial.available()) Serial.read();

        Serial.print("Executing command: ");
        
        switch (cmd) {
            case 't':
                Serial.println("takeoff");
                status = tello.takeoff();
                break;
            case 'l':
                Serial.println("land");
                status = tello.land();
                break;
            case 'w':
                Serial.println("up 30cm");
                status = tello.move_up(30);
                break;
            case 's':
                Serial.println("down 30cm");
                status = tello.move_down(30);
                break;
            case 'a':
                Serial.println("left 30cm");
                status = tello.move_left(30);
                break;
            case 'd':
                Serial.println("right 30cm");
                status = tello.move_right(30);
                break;
            case 'i':
                Serial.println("forward 30cm");
                status = tello.move_forward(30);
                break;
            case 'k':
                Serial.println("back 30cm");
                status = tello.move_back(30);
                break;
            case 'j':
                Serial.println("rotate left 30°");
                status = tello.rotate_counter_clockwise(30);
                break;
            case 'r':
                Serial.println("rotate right 30°");
                status = tello.rotate_clockwise(30);
                break;
            case 'q':
                Serial.println("quitting...");
                if (tello.isFlying()) {
                    tello.land();
                    delay(3000);
                }
                tello.disconnect();
                Serial.println("Disconnected. Program ended.");
                while (true);
            case '?':
                printCommands();
                return;
            default:  // Invalid command
                Serial.println("Invalid command. Press '?' for help.");
                return;
        }

        // Error status handling
        if (status != TelloStatus::OK) {
            switch(status) {
                case TelloStatus::Timeout: Serial.println("Command failed: Timeout"); break;
                case TelloStatus::NotConnected: Serial.println("Command failed: Not Connected"); break;
                case TelloStatus::NoResponse: Serial.println("Command failed: No Response"); break;
                case TelloStatus::InvalidParameter: Serial.println("Command failed: Invalid Parameter"); break;
                default: break; // Don't handle other cases as they're covered by the callback
            }
        }
    }

    // Update status every second
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate >= 1000) {
        if (tello.isConnected()) {
            Serial.printf("Status - Battery: %d%%, Height: %d cm, Flight time: %d s\n",
                          tello.get_battery(), tello.get_height(), tello.get_flight_time());
        }
        lastUpdate = millis();
    }
}
