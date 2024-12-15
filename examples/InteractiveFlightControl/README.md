# Interactive Flight Control Example

This example allows real-time control of the Tello drone using keyboard commands via the Serial Monitor. It provides continuous feedback about the drone's status including battery level, height, and flight time.

Source code: https://github.com/sagar-koirala/TelloESP32.git

## Features
- Interactive keyboard control
- Real-time status updates
- Error handling and feedback
- Safe disconnection procedure

## Available Commands
| Key | Action |
|-----|--------|
| t | Take off |
| l | Land |
| w | Move up 30cm |
| s | Move down 30cm |
| a | Move left 30cm |
| d | Move right 30cm |
| i | Move forward 30cm |
| k | Move back 30cm |
| j | Rotate left 30° |
| r | Rotate right 30° |
| q | Quit program |
| ? | Show command list |

## Expected Serial Output
When running correctly, you should see output similar to this (timestamps will vary):
```
13:03:51.717 -> TelloESP32 Interactive Flight Control Example
13:04:02.233 -> Connected to Tello successfully!
13:04:02.233 -> 
13:04:02.233 -> Available Commands:
13:04:02.233 -> t - takeoff
13:04:02.233 -> l - land
13:04:02.233 -> w - up 30cm
13:04:02.233 -> s - down 30cm
13:04:02.233 -> a - left 30cm
13:04:02.233 -> d - right 30cm
13:04:02.233 -> i - forward 30cm
13:04:02.233 -> k - back 30cm
13:04:02.233 -> j - rotate left 30°
13:04:02.233 -> r - rotate right 30°
13:04:02.233 -> q - quit program

13:04:02.233 -> Status - Battery: 0%, Height: 0 cm, Flight time: 0 s
13:04:03.787 -> Executing command: takeoff
13:04:10.008 -> Status - Battery: 81%, Height: 60 cm, Flight time: 5 s
13:04:10.927 -> Executing command: rotate left 30°
13:04:12.128 -> Status - Battery: 81%, Height: 60 cm, Flight time: 7 s
13:04:12.172 -> Executing command: rotate left 30°
13:04:13.367 -> Status - Battery: 80%, Height: 60 cm, Flight time: 9 s
13:04:13.616 -> Executing command: rotate right 30°
13:04:14.909 -> Status - Battery: 80%, Height: 60 cm, Flight time: 10 s
13:04:14.909 -> Executing command: rotate right 30°
13:04:16.433 -> Status - Battery: 79%, Height: 60 cm, Flight time: 12 s
13:04:16.479 -> Executing command: rotate right 30°
13:04:17.998 -> Status - Battery: 78%, Height: 60 cm, Flight time: 14 s
13:04:17.998 -> Executing command: rotate right 30°
13:04:19.499 -> Status - Battery: 77%, Height: 60 cm, Flight time: 15 s
13:04:19.966 -> Executing command: rotate left 30°
13:04:21.677 -> Status - Battery: 76%, Height: 60 cm, Flight time: 17 s
13:04:22.094 -> Executing command: left 30cm
13:04:23.834 -> Status - Battery: 75%, Height: 60 cm, Flight time: 19 s
13:04:24.157 -> Executing command: right 30cm
13:04:25.933 -> Status - Battery: 74%, Height: 60 cm, Flight time: 21 s
13:04:26.557 -> Executing command: down 30cm
13:04:29.661 -> Status - Battery: 72%, Height: 30 cm, Flight time: 25 s
13:04:29.707 -> Executing command: up 30cm
13:04:31.483 -> Status - Battery: 71%, Height: 60 cm, Flight time: 27 s
13:04:33.057 -> Executing command: back 30cm
13:04:34.869 -> Status - Battery: 70%, Height: 60 cm, Flight time: 30 s
13:04:35.896 -> Status - Battery: 70%, Height: 60 cm, Flight time: 31 s
13:04:36.474 -> Executing command: forward 30cm
13:04:38.257 -> Status - Battery: 69%, Height: 70 cm, Flight time: 34 s
13:04:38.708 -> Executing command: back 30cm
13:04:40.732 -> Status - Battery: 69%, Height: 60 cm, Flight time: 36 s
13:04:41.150 -> Executing command: rotate left 30°
13:04:42.234 -> Status - Battery: 68%, Height: 60 cm, Flight time: 38 s
13:04:43.177 -> Executing command: land
13:04:47.166 -> Status - Battery: 67%, Height: 0 cm, Flight time: 43 s
13:04:51.303 -> Executing command: quitting...
13:04:51.303 -> E (66705) wifi:NAN WiFi stop
13:04:51.341 -> Disconnected. Program ended.
```

## Usage Notes
1. Set your Serial Monitor baud rate to 115200
2. Enter commands as single letters (lowercase)
3. Watch the status updates for battery level and height
4. Use 'q' to safely land and disconnect

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