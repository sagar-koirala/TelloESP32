# Get Drone State Example

This example demonstrates how to retrieve and display real-time telemetry data from the Tello drone using the TelloESP32 library.

Source code: https://github.com/sagar-koirala/TelloESP32.git

## About State Updates
The `get_` functions (like `get_battery()`, `get_height()`) provide immediate access to the drone's state data that is automatically updated in the background. These functions are:
- Much faster than query commands
- Don't generate network traffic
- Update automatically every ~100ms
- Ideal for frequent status checks

## Displayed Information
- Battery level
- Height
- Flight time
- Temperature
- Barometer reading
- TOF (Time of Flight) distance
- Orientation (Pitch, Roll, Yaw)
- Speed (X, Y, Z axes)
- Acceleration (X, Y, Z axes)

## Expected Serial Output
When running correctly, you should see output similar to this (timestamps will vary):
```
10:15:23.372 -> TelloESP32 Get Drone State Example
10:15:25.491 -> Connected to Tello successfully!

10:15:26.524 -> --- Drone State ---
10:15:26.524 -> Battery: 87%
10:15:26.524 -> Height: 0 cm
10:15:26.524 -> Flight time: 0 s
10:15:26.524 -> Temperature: 24.5°C
10:15:26.524 -> Barometer: 100.25 m
10:15:26.524 -> TOF Distance: 120.0 cm
10:15:26.524 -> Pitch: 1°
10:15:26.524 -> Roll: 0°
10:15:26.524 -> Yaw: 88°
10:15:26.524 -> Speed - X: 0, Y: 0, Z: 0 cm/s
10:15:26.524 -> Acceleration - X: 0.00, Y: 0.00, Z: 0.00 cm/s²
10:15:26.524 -> ------------------
```

## Usage Notes
1. Set your Serial Monitor baud rate to 115200
2. Updates occur every second while connected
3. The program will stop if connection is lost

## Available Functions
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