#include <WiFi.h>
#include "TelloESP32.h"
#include <SD_MMC.h>  // Use SD_MMC library

using namespace TelloControl;

// Tello drone credentials
const char *TELLO_SSID = "TELLO-56CC13"; // Replace with your Tello's SSID
const char *TELLO_PASSWORD = "";

TelloESP32 tello;
unsigned long frameCount = 0;

// Callback function for video data
void onVideoData(const uint8_t *data, size_t length) {
    frameCount++;
    Serial.printf("Received video frame %lu, size: %u bytes\n", frameCount, length);
    // Here you could process the video data or forward it to another device
}

void setup() {
    Serial.begin(115200);
    Serial.println("TelloESP32 Video Stream Example");

    // Initialize SD card using SD_MMC
    if (!SD_MMC.begin()) {
        Serial.println("Failed to initialize SD card");
        return;
    }

    // Connect to Tello drone
    if (tello.connect(TELLO_SSID, TELLO_PASSWORD) != TelloStatus::OK) {
        Serial.println("Connection failed!");
        return;
    }

    // Set video configuration
    tello.set_video_direction(CameraDirection::FORWARD);
    tello.set_video_resolution(VideoResolution::RESOLUTION_720P);
    tello.set_video_fps(VideoFPS::FPS_30);
    tello.set_video_bitrate(VideoBitrate::BITRATE_4MBPS);

    // Register video callback
    tello.setVideoDataCallback(onVideoData);

    // Start video stream
    if (tello.streamon() == TelloStatus::OK) {
        Serial.println("Video stream started!");
        delay(5000);
        tello.streamoff();
        tello.disconnect();
    }
}

void loop() {
    // nothing here
}