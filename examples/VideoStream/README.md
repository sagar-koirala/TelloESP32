# Video Stream Example

This example demonstrates how to receive the video stream from a Tello drone using the TelloESP32 library.

Source code: https://github.com/sagar-koirala/TelloESP32.git

## Usage Notes
1. Set your Serial Monitor baud rate to 115200
2. Make sure you have enough RAM available on your ESP32 
3. The video stream callback function is called in background, irrespective of the task on the loop function
4. The program will stop if connection is lost

## Video Stream Details
The Tello drone streams H.264 encoded video. Please note that:
- ESP32 cannot decode H.264 video in real-time
- This example only demonstrates receiving raw video frames
- To view the video, you need to forward the stream to a more capable device

## Video Configuration
The example configures these video parameters:
- Direction: Forward-facing camera
- Resolution: 720p
- Frame Rate: 30 FPS
- Bitrate: 4 Mbps

## Expected Serial Output
When running correctly, you should see output similar to this (timestamps will vary):
```
10:15:23.372 -> TelloESP32 Video Stream Example
10:15:25.491 -> Connected to Tello successfully!
10:15:26.524 -> Video stream started!
10:15:26.624 -> Received video frame 1, size: 8192 bytes
10:15:26.724 -> Received video frame 2, size: 7168 bytes
10:15:27.524 -> Status - Frames: 2, Battery: 87%, Height: 0 cm
10:15:27.624 -> Received video frame 3, size: 6144 bytes
```

## Practical Applications
You can use this example to:
1. Forward the stream to another device
2. Save raw video data for later processing
3. Monitor video frame rates and sizes

## Available Functions
### Video Configuration

*   **`TelloStatus set_video_bitrate(VideoBitrate bitrate)`:** Set the video bitrate.
*   **`TelloStatus set_video_resolution(VideoResolution resolution)`:** Set the video resolution.
*   **`TelloStatus set_video_fps(VideoFPS fps)`:** Set the video FPS (frames per second).

### Video Streaming

*   **`TelloStatus streamon()`:** Start video streaming.
*   **`TelloStatus streamoff()`:** Stop video streaming.
*   **`typedef void (*VideoDataCallback)(const uint8_t *data, size_t length)`:**  The function pointer type for the video data callback.
*   **`void setVideoDataCallback(VideoDataCallback callback)`:** Sets the callback function that will be called when new video data is received.
*   **`bool startVideoRecording(const char* filename)`:** Start recording video and save to the file specified by filename.

### Camera Control

*   **`TelloStatus set_video_direction(CameraDirection direction)`:** Set the camera direction (forward or downward).