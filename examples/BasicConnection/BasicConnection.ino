/*
  BasicConnection.ino - Demonstrates establishing a basic connection to a DJI Tello drone.

  This example shows the minimal code needed to connect to a Tello drone using the 
  TelloESP32 library. It connects to the drone's WiFi network, initializes the 
  communication protocol (SDK mode), and then disconnects. Use this as a starting 
  point to verify your ESP32 can communicate with your Tello.
  Source code: https://github.com/sagar-koirala/TelloESP32.git
  API reference: https://github.com/sagar-koirala/TelloESP32?tab=readme-ov-file#connection-and-initialization
*/

#include "TelloESP32.h" // Include the TelloESP32 library

using namespace TelloControl;

const char* TELLO_SSID = "TELLO-56CC13";  // Replace with your Tello's SSID
const char* TELLO_PASSWORD = "";          // Tello's WiFi password (usually empty)

TelloESP32 tello;

void setup() {
  Serial.begin(115200); // Initialize serial communication for debugging
  Serial.println("TelloESP32 Basic Connection Example");

  Serial.print("Connecting to Tello drone: ");
  Serial.println(TELLO_SSID);

  // Connect to the Tello drone's WiFi network
  TelloStatus status = tello.connect(TELLO_SSID, TELLO_PASSWORD);

  if (status == TelloStatus::OK) {
    Serial.println("Successfully connected to Tello!");

    delay(5000);

    Serial.println("Disconnecting from Tello...");
    tello.disconnect();
    Serial.println("Disconnected.");
  } else {
    Serial.print("Error connecting to Tello: ");
    Serial.println(static_cast<int>(status)); // Print the error code
  }
}

void loop() {
  // All operations are in setup()
}