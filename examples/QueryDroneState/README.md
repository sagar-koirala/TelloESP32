# Query Drone State Example

This example demonstrates how to query various state information directly from the Tello drone using specific query commands. Unlike the GetDroneState example which uses state updates, this example actively queries the drone for information.

Source code: https://github.com/sagar-koirala/TelloESP32.git

## Features
- Direct command queries
- Comprehensive status information
- Error handling with callback
- Regular status updates

## About Query Commands
The `query_` functions (like `query_battery()`, `query_height()`) send direct commands to the drone. These functions:
- Get real-time data directly from the drone
- Generate network traffic with each query
- Take longer to execute than get functions
- Useful when you need guaranteed current values
- Should be used sparingly (2+ second intervals recommended)

## Queried Information
- Battery level
- Height
- Flight time
- Speed
- Temperature
- Barometer reading
- TOF Distance
- WiFi SNR
- SDK Version
- Serial Number

## Expected Serial Output
When running correctly, you should see output similar to this (timestamps will vary):
```
14:36:28.397 -> TelloESP32 Query Drone State Example
14:36:46.601 -> Connected to Tello successfully!
14:36:46.601 -> 
14:36:46.601 -> --- Query Results ---
14:36:47.117 -> Battery: 89%
14:36:47.567 -> Height: 0 cm
14:36:48.098 -> Flight time: 0 s
14:36:48.609 -> Speed: 100 cm/s
14:36:49.122 -> Temperature: 54°C
14:36:49.634 -> Barometer: 48.31 m
14:36:50.083 -> TOF Distance: 100.0 cm
14:36:50.610 -> WiFi SNR: 90
14:36:50.610 -> 
14:36:51.122 -> SDK Version: 30
14:36:51.621 -> Serial Number: 0TQDG66EDBB3NU
14:36:51.621 -> ------------------
```

## Usage Notes
1. Set your Serial Monitor baud rate to 115200
2. Updates occur every 2 seconds while connected
3. The program will stop if connection is lost
4. Query commands are direct requests to the drone

## Available Functions
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