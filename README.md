# TelloESP32: Arduino Library for Controlling DJI Tello Drones with ESP32

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](https://opensource.org/licenses/MIT)

## Overview

Control your DJI Tello drone with an ESP32 microcontroller using this easy-to-use Arduino library!  Inspired by the [`djitellopy`](https://github.com/damiafuentes/DJITelloPy) Python library, [`TelloESP32`]()brings similar functionality to the Arduino ecosystem, enabling embedded Tello applications.

**Key Features:**

* **Simplified C++ API:** Control flight, video, and drone state with a clean C++ interface designed for Arduino.
* **ESP32 Optimized:** Leverages the ESP32's dual-core processor and WiFi capabilities for efficient communication and video processing.
* **Flight Control:** Take off, land, move, rotate, and flip with simple functions.
* **Video Streaming:** Receive and process live H.264 video data with a callback function. *(Note:  Resource intensive; consider lower resolutions/framerates)*.
* **SD Card Recording:** Save the streamed H264 video to a mounted SD card.
* **State Monitoring:** Access real-time telemetry like battery level, altitude, speed, temperature, and IMU data. Direct state access (faster) and query commands are provided.
* **Mission Pad Support (Tello EDU):**  Interface with Tello mission pads for programmed flight.
* **Network Configuration (Tello EDU):**  Change the Tello's WiFi SSID and password.
* **Expansion Board Commands:** Send custom commands to compatible expansion boards.

## Getting Started

## Installation

1. **Arduino Library Manager (Recommended):** Search for "TelloESP32" in the Library Manager and install.
2. **Manual:** [Download](https://github.com/sagar-koirala/TelloESP32/archive/refs/heads/master.zip) the latest release ZIP, rename the folder to `TelloESP32`, and place it in your Arduino `libraries` directory.

## Examples

The [`examples`](./examples/)  folder contains practical demonstrations of key library features:

* **Basic Flight:**  Simple takeoff, movement, and landing.
* **Video Streaming:** Receiving video data and (optionally) displaying it or processing it.
* **SD Card Recording:** Record and save your video stream.

## Usage

### Basic Connection

Here's how to connect to your Tello drone:

```cpp
#include <TelloESP32.h>

using namespace TelloControl;

TelloESP32 tello;

void setup() {
  Serial.begin(115200);

  // Connect to the Tello's WiFi network
  TelloStatus status = tello.connect("TELLO-XXXXXX", ""); // Replace with your Tello's SSID and password if needed

  if (status == TelloStatus::OK) {
    Serial.println("Connected to Tello!");

    // You can start sending commands now
    tello.takeoff();
  } else {
    Serial.print("Error connecting to Tello: ");
    Serial.println((int)status);
  }
}

void loop() {
  // Your main loop code (e.g., reading sensor data, sending RC commands)
}
```

**Explanation:**

1. **Include:** Include the `TelloESP32.h` header file.
2. **Namespace:** Use the `TelloControl` namespace to avoid naming conflicts.
3. **Object:** Create a `TelloESP32` object (e.g., `tello`).
4. **Serial Monitor:** Initialize the serial monitor for debugging (optional but recommended).
5. **Connect:** Use the `connect()` method to connect to the Tello's WiFi network. Replace `"TELLO-XXXXXX"` with your drone's SSID (found on the battery compartment) and leave the password empty unless you've set one on your drone.
6. **Status Check:** Check the returned `TelloStatus` to ensure the connection was successful.

### Sending Commands

Use the various command methods provided by the library to control the drone:

```cpp
// ... (after successful connection)

tello.takeoff();
delay(5000);          // Wait 5 seconds
tello.move_forward(50); // Move forward 50 cm
delay(3000);
tello.rotate_clockwise(90); // Rotate 90 degrees clockwise
delay(3000);
tello.land();
```

### Receiving State Data

You can access the drone's state information in two ways:

**1. Query Commands:**

```cpp
int battery = tello.query_battery();
Serial.print("Battery: ");
Serial.print(battery);
Serial.println("%");

int height = tello.query_height();
Serial.print("Height: ");
Serial.print(height);
Serial.println(" cm");
```

**2. Direct State Getters (Faster):**

```cpp
int battery = tello.get_battery();
Serial.print("Battery: ");
Serial.print(battery);
Serial.println("%");

int height = tello.get_height();
Serial.print("Height: ");
Serial.print(height);
Serial.println(" cm");
```

### Video Streaming

```cpp
#include <TelloESP32.h>

using namespace TelloControl;

TelloESP32 tello;

// Video data callback function
void videoDataCallback(const uint8_t *data, size_t length) {
  // Process the video data here (e.g., send it over serial, display it on a screen)
  Serial.printf("Received video data: %u bytes\n", length);

  // Example: Print the first 10 bytes of the video frame
  for (int i = 0; i < 10 && i < length; i++) {
    Serial.printf("%02X ", data[i]);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  
  // Connect to Tello
  if (tello.connect("TELLO-XXXXXX", "") == TelloStatus::OK) {
    Serial.println("Connected to Tello!");

    // Set the video data callback function
    tello.setVideoDataCallback(videoDataCallback);

    // Start video streaming
    tello.streamon();
  } else {
    Serial.println("Failed to connect to Tello");
  }
}

void loop() {
  // Your main loop code can go here
}
```

## API Reference

### Connection and Initialization

*   **`TelloESP32()`:** Constructor. Initializes the object.
*   **`~TelloESP32()`:** Destructor. Cleans up resources.
*   **`TelloStatus connect(const char *ssid, const char *password, unsigned long timeout_ms = 10000)`:** Connects to the Tello's WiFi network.
    *   `ssid`: The Tello's SSID (e.g., "TELLO-XXXXXX").
    *   `password`: The Tello's WiFi password (usually empty).
    *   `timeout_ms`: Connection timeout in milliseconds.
    *   Returns: `TelloStatus` indicating success or failure.
*   **`void disconnect()`:** Disconnects from the Tello.
*   **`bool isConnected() const`:** Returns `true` if connected, `false` otherwise.

### Flight Control

*   **`TelloStatus takeoff()`:** Initiates takeoff.
*   **`TelloStatus land()`:** Initiates landing.
*   **`bool isFlying() const`:** Returns `true` if the drone is in the air, `false` otherwise.

### Movement Commands

*   **`TelloStatus move_up(int cm)`:** Move up by `cm` (20-500).
*   **`TelloStatus move_down(int cm)`:** Move down by `cm` (20-500).
*   **`TelloStatus move_left(int cm)`:** Move left by `cm` (20-500).
*   **`TelloStatus move_right(int cm)`:** Move right by `cm` (20-500).
*   **`TelloStatus move_forward(int cm)`:** Move forward by `cm` (20-500).
*   **`TelloStatus move_back(int cm)`:** Move back by `cm` (20-500).
*   **`TelloStatus rotate_clockwise(int degrees)`:** Rotate clockwise by `degrees` (1-360).
*   **`TelloStatus rotate_counter_clockwise(int degrees)`:** Rotate counter-clockwise by `degrees` (1-360).

### Flip Commands

*   **`TelloStatus flip_left()`:** Flip to the left.
*   **`TelloStatus flip_right()`:** Flip to the right.
*   **`TelloStatus flip_forward()`:** Flip forward.
*   **`TelloStatus flip_back()`:** Flip backward.

### Motor Control

*   **`TelloStatus turn_motor_on()`:** Turns the motors on (for testing on the ground, not for flying).
*   **`TelloStatus turn_motor_off()`:** Turns the motors off.

### Advanced Movement

*   **`TelloStatus go_xyz_speed(int x, int y, int z, int speed)`:** Move to coordinates `x`, `y`, `z` at `speed` (x, y, z: -500 to 500, speed: 10-100).
*   **`TelloStatus curve_xyz_speed(int x1, int y1, int z1, int x2, int y2, int z2, int speed)`:** Fly a curve from (x1, y1, z1) to (x2, y2, z2) at `speed` (x1, y1, z1, x2, y2, z2: -500 to 500, speed: 10-60).
*   **`TelloStatus set_speed(int speed)`:** Set the drone's speed (10-100 cm/s).
*   **`void send_rc_control(int left_right_velocity, int forward_backward_velocity, int up_down_velocity, int yaw_velocity)`:** Send RC control commands (-100 to 100 for each parameter).

### Query Commands

*   **`int query_speed() const`:** Get the current speed setting.
*   **`int query_battery() const`:** Get the current battery percentage.
*   **`int query_time() const`:** Get the current flight time.
*   **`int query_height() const`:** Get the current height (in dm).
*   **`int query_temp() const`:** Get the current temperature (average).
*   **`float query_attitude() const`:** Get the current attitude (pitch, roll, yaw).
*   **`float query_baro() const`:** Get the barometer reading.
*   **`float query_tof() const`:** Get the time-of-flight distance.
*   **`String query_wifi() const`:** Get the WiFi SNR (signal-to-noise ratio).
*   **`String query_sdk() const`:** Get the Tello SDK version.
*   **`String query_sn() const`:** Get the Tello serial number.

### Direct State Getters

*   **`int get_height() const`:** Get the current height in cm.
*   **`int get_battery() const`:** Get the current battery percentage.
*   **`int get_flight_time() const`:** Get the current flight time in seconds.
*   **`float get_temperature() const`:** Get the current temperature in Celsius.
*   **`float get_barometer() const`:** Get the current barometer reading in meters.
*   **`float get_tof() const`:** Get the current TOF (time-of-flight) distance in cm.
*   **`int get_pitch() const`:** Get the current pitch in degrees.
*   **`int get_roll() const`:** Get the current roll in degrees.
*   **`int get_yaw() const`:** Get the current yaw in degrees.
*   **`int get_speed_x() const`:** Get the current x-axis speed in cm/s.
*   **`int get_speed_y() const`:** Get the current y-axis speed in cm/s.
*   **`int get_speed_z() const`:** Get the current z-axis speed in cm/s.
*   **`float get_acceleration_x() const`:** Get the current x-axis acceleration in cm/s².
*   **`float get_acceleration_y() const`:** Get the current y-axis acceleration in cm/s².
*   **`float get_acceleration_z() const`:** Get the current z-axis acceleration in cm/s².

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

### Mission Pad

*   **`TelloStatus enable_mission_pads()`:** Enable mission pad detection.
*   **`TelloStatus disable_mission_pads()`:** Disable mission pad detection.
*   **`TelloStatus set_mission_pad_detection_direction(int direction)`:** Set the direction for mission pad detection (0: both, 1: downward, 2: forward).

### Network Configuration

*   **`TelloStatus set_wifi_credentials(const char *ssid, const char *password)`:** Set the WiFi credentials for the Tello to connect to a different network (AP mode).
*   **`TelloStatus connect_to_wifi(const char *ssid, const char *password)`:** Connect the Tello to a specified WiFi network in station mode.

### Camera Control

*   **`TelloStatus set_video_direction(CameraDirection direction)`:** Set the camera direction (forward or downward).

### Expansion Board

*   **`TelloStatus send_expansion_command(const char *cmd)`:** Send a custom command to the Tello expansion board.

## Troubleshooting

* **Connection problems?** Double-check your Tello SSID and WiFi credentials. Restart your ESP32 and the Tello. Ensure a strong WiFi signal.
* **Commands not working?** Make sure you call `connect()` successfully. Verify the returned `TelloStatus`. Ensure there's a sufficient delay (e.g., 500ms) between commands.
* **Video stream issues?**  Confirm `streamon()` is called and that your `setVideoDataCallback()` function is correct. Video processing within the callback should be as fast as possible to avoid blocking other operations. Lower resolutions and frame rates may be necessary on the ESP32.

## Contributing

Contributions are welcome! Please follow the existing coding style and submit pull requests.

## References

* [Tello SDK 1.3.0](https://dl-cdn.ryzerobotics.com/downloads/tello/20180910/Tello%20SDK%20Documentation%20EN_1.3.pdf)
* [Tello SDK 2.0](https://dl-cdn.ryzerobotics.com/downloads/Tello/Tello%20SDK%202.0%20User%20Guide.pdf)
* [DJITelloPy](https://github.com/damiafuentes/DJITelloPy)