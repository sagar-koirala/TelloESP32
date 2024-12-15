/* 
  VideoStream.ino - Example code demonstrating how to receive video stream
  from a DJI Tello drone using the TelloESP32 library.
  Source code: https://github.com/sagar-koirala/TelloESP32.git
  API reference: https://github.com/sagar-koirala/TelloESP32?tab=readme-ov-file#video-configuration
*/

#include "TelloESP32.h"

using namespace TelloControl;

const char *TELLO_SSID = "TELLO-56CC13";  // Replace with your Tello's SSID
const char *TELLO_PASSWORD = "";          // Tello's WiFi password (usually empty)

TelloESP32 tello;
unsigned long frameCount = 0;

// Error handler callback function
void telloErrorHandler(const char* command, const char* errorMessage) {
    Serial.printf("[ERROR] Command '%s': %s\n", command, errorMessage);
}

// Callback function for video data
void onVideoData(const uint8_t *data, size_t length) {
    frameCount++;
    Serial.printf("Received video frame %lu, size: %u bytes\n", frameCount, length);
}

void setup() {
    Serial.begin(115200);
    Serial.println("TelloESP32 Video Stream Example");
    
    // Set the error handler callback
    tello.setErrorCallback(telloErrorHandler);

    // Connect to Tello drone
    TelloStatus status = tello.connect(TELLO_SSID, TELLO_PASSWORD);
    if (status != TelloStatus::OK) {
        Serial.println("Connection failed!");
        return;
    }
    Serial.println("Connected to Tello successfully!");

    // Configure video settings
    tello.set_video_direction(CameraDirection::FORWARD);
    tello.set_video_resolution(VideoResolution::RESOLUTION_720P);
    tello.set_video_fps(VideoFPS::FPS_30);
    tello.set_video_bitrate(VideoBitrate::BITRATE_4MBPS);

    // Register video callback and start stream
    tello.setVideoDataCallback(onVideoData);
    if (tello.streamon() == TelloStatus::OK) {
        Serial.println("Video stream started!");
    } else {
        Serial.println("Failed to start video stream!");
    }
}

void loop() {
    static unsigned long lastUpdate = 0;
    
    if (tello.isConnected() && (millis() - lastUpdate >= 1000)) {
        // Print status every second
        Serial.printf("Status - Frames: %lu, Battery: %d%%, Height: %d cm\n",
                      frameCount, tello.get_battery(), tello.get_height());
        lastUpdate = millis();
    } else if (!tello.isConnected()) {
        Serial.println("Connection lost!");
        while (true); // Stop execution
    }
}