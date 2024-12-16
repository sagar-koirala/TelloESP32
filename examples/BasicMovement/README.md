# Basic Movement Example

This example demonstrates fundamental flight movements of the Tello drone using the TelloESP32 library.

Source code: https://github.com/sagar-koirala/TelloESP32.git

## Flight Sequence
The drone will perform the following movements:
1. Take off
2. Move in a square pattern (30cm per side)
3. Rotate 360 degrees
4. Land safely

## Usage Notes
1. Replace `TELLO_SSID` with your Tello's SSID if different
2. Make sure the Drone is in a open space with good lighting condition and has enough battery charge
3. Set your Serial Monitor baud rate to 115200 to check the debug messages

## Expected Serial Output
When running correctly, you should see output similar to this (timestamps will vary):
```
09:37:15.372 -> TelloESP32 Basic Movement Example
09:37:25.491 -> Connected! Starting flight sequence...
09:37:32.024 -> 1. Taking off...
09:37:34.068 -> 2. Moving in square pattern...
09:37:50.330 -> 3. Rotating 360 degrees...
09:37:58.888 -> 4. Landing...
09:38:01.483 -> Flight sequence completed successfully!
09:38:01.483 -> E (58420) wifi:NAN WiFi stop
```

## Available Functions
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