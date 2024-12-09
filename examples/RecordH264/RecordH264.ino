#include <WiFi.h>
#include "TelloESP32.h"
#include <SD_MMC.h>  // Use SD_MMC library instead of SD.h

using namespace TelloControl;

const char *TELLO_SSID = "TELLO-56CC13"; // Replace with your Tello's SSID
const char *TELLO_PASSWORD = "";

TelloESP32 tello;
File videoFile;
bool isRecording = false;
unsigned long frameCount = 0;
unsigned long totalBytes = 0;
unsigned long recordingStartTime = 0;

// Function to generate unique filename
String getNextFileName() {
    int fileIndex = 0;
    String fileName;
    do {
        fileName = "/tello_" + String(fileIndex) + ".h264";
        fileIndex++;
    } while (SD_MMC.exists(fileName));
    return fileName;
}

// Callback function for video data
void onVideoData(const uint8_t *data, size_t length) {
    if (!isRecording || !videoFile) return;
    
    size_t bytesWritten = videoFile.write(data, length);
    if (bytesWritten != length) {
        Serial.println("Error writing to file!");
        return;
    }
    
    frameCount++;
    totalBytes += bytesWritten;
    
    // Print statistics every 30 frames
    if (frameCount % 30 == 0) {
        float duration = (millis() - recordingStartTime) / 1000.0;
        float kbps = (totalBytes * 8.0) / (duration * 1024.0);
        Serial.printf("Recording: %lu frames, %.2f KB/s\n", frameCount, kbps);
    }
}

void startRecording() {
    if (isRecording) return;

    String fileName = getNextFileName();
    videoFile = SD_MMC.open(fileName, FILE_WRITE);
    if (!videoFile) {
        Serial.println("Failed to create file!");
        return;
    }

    isRecording = true;
    frameCount = 0;
    totalBytes = 0;
    recordingStartTime = millis();
    Serial.printf("Started recording to %s\n", fileName.c_str());
}

void stopRecording() {
    if (!isRecording) return;

    isRecording = false;
    if (videoFile) {
        videoFile.close();
        float duration = (millis() - recordingStartTime) / 1000.0;
        Serial.printf("Recording stopped. %lu frames, %.1f seconds\n", 
                     frameCount, duration);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("TelloESP32 H264 Recording Example");
    
    // Initialize SD card using SD_MMC
    if (!SD_MMC.begin()) {
        Serial.println("Failed to mount SD card");
        return;
    }
    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);

    // Connect to Tello
    if (tello.connect(TELLO_SSID, TELLO_PASSWORD) != TelloStatus::OK) {
        Serial.println("Connection failed!");
        return;
    }

    // Configure video settings
    tello.set_video_resolution(VideoResolution::RESOLUTION_720P);
    tello.set_video_fps(VideoFPS::FPS_30);
    tello.set_video_bitrate(VideoBitrate::BITRATE_4MBPS);

    // Register video callback and start stream
    tello.setVideoDataCallback(onVideoData);
    if (tello.streamon() != TelloStatus::OK) {
        Serial.println("Failed to start video stream!");
        return;
    }

    Serial.println("Ready for recording!");
    Serial.println("Commands: 'r' to start/stop recording, 'q' to quit");
}

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();
        switch (cmd) {
            case 'r':
                if (isRecording) {
                    stopRecording();
                } else {
                    startRecording();
                }
                break;
                
            case 'q':
                stopRecording();
                tello.streamoff();
                tello.disconnect();
                Serial.println("Disconnected");
                while(1);
                break;
        }
    }
}