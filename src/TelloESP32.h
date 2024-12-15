/*!
 * TelloESP32.h - Library for controlling DJI Ryze Tello drones using ESP32
 * Created by Sagar Koirala, 12-2024
 * 
 * This library provides a comprehensive interface for controlling Tello drones
 * using ESP32 microcontrollers. It includes:
 * - Basic flight controls (takeoff, land, movement)
 * - Video streaming and recording
 * - State monitoring (battery, height, speed, etc.)
 * - Mission pad detection (Tello EDU)
 * - WiFi configuration (Tello EDU)
 * 
 * Source code: https://github.com/sagar-koirala/TelloESP32.git
 * API reference: https://github.com/sagar-koirala/TelloESP32?tab=readme-ov-file#api-reference
 * 
 * Usage Notes:
 * - Always check connection status before sending commands.
 * - Allow sufficient delay between movements (500ms recommended).
 * - Monitor battery levels during flight.
 * - Ensure proper error handling.
 * - Use direct state getters for frequent status checks.
 * - Query commands should be used sparingly.
 * 
 * License: MIT License (See accompanying LICENSE file or https://opensource.org/licenses/MIT) 
 */

#ifndef TELLOESP32_H
#define TELLOESP32_H

// State message debug control
#ifndef print_Raw_State
#define print_Raw_State 0 // Set to 1 to enable state message debug output
#endif

// Debug logging control
#ifndef TELLO_DEBUG
#define TELLO_DEBUG 0  // Set to 1 to enable debug output
#endif

#if TELLO_DEBUG
    #define TELLO_LOG(x)    Serial.print(x)
    #define TELLO_LOGLN(x)  Serial.println(x)
    #define TELLO_LOGF(...) Serial.printf(__VA_ARGS__)
#else
    #define TELLO_LOG(x)    
    #define TELLO_LOGLN(x)  
    #define TELLO_LOGF(...) 
#endif

// --- Standard Libraries ---
// Note: Including <string> is not needed as Arduino's String class is used and it is usually included by default.
// If you are not using the Arduino environment and need std::string, then you would include <string>.
#include <WiFi.h>
#include <WiFiUdp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Define constants for Tello ports
#define TELLO_CMD_PORT  8889    // Tello command port
#define TELLO_STATE_PORT 8890   // Tello state port
#define TELLO_VIDEO_PORT 11111  // Tello video stream port

// Configuration options (can be changed by the user)
#define MAX_RESPONSE_LENGTH 128 // Maximum length of a single response string

namespace TelloControl {

// --- Enums ---
// Status codes for command execution results
enum class TelloStatus {
    OK,
    Timeout,
    ConnectionLost,
    NotConnected,
    NoResponse,
    InvalidParameter,
    UdpError,
    UnknownError
};

// Video configuration enums
enum class VideoBitrate {
    BITRATE_AUTO,
    BITRATE_1MBPS,
    BITRATE_2MBPS,
    BITRATE_3MBPS,
    BITRATE_4MBPS,
    BITRATE_5MBPS
};

enum class VideoResolution {
    RESOLUTION_720P,
    RESOLUTION_480P
};

enum class VideoFPS {
    FPS_5,
    FPS_15,
    FPS_30
};

enum class CameraDirection {
    FORWARD = 0,
    DOWNWARD = 1
};

enum class CommandType {
    Action,
    Query
};

// --- TelloESP32 Class ---
/**
 * @brief Main class for controlling the Tello drone.
 */
class TelloESP32 {
public:
    // ----------------------------------
    // Constructor and Destructor
    // ----------------------------------

    /**
     * @brief Constructor for TelloESP32 class.
     * Initializes default values for network addresses, ports, and internal state variables.
     */
    TelloESP32();

    /**
     * @brief Destructor for TelloESP32 class.
     * Cleans up resources, stops tasks, and closes UDP sockets.
     */
    ~TelloESP32();

    // ----------------------------------
    // Connection Management
    // ----------------------------------

    /**
     * @brief Connect to Tello drone.
     * @param ssid The SSID of the Tello drone.
     * @param password The password (empty for Tello, unless configured for station mode).
     * @param timeout_ms Connection timeout in milliseconds.
     * @return Status of the connection attempt.
     */
    TelloStatus connect(const char *ssid, const char *password, unsigned long timeout_ms = 10000);

    /**
     * @brief Disconnect from the Tello drone.
     * Stops any running tasks and closes UDP sockets.
     */
    void disconnect();

    /**
     * @brief Check if the ESP32 is currently connected to the Tello drone.
     * @return True if connected, false otherwise.
     */
    bool isConnected() const { return connected; }

    // ----------------------------------
    // Command Sending
    // ----------------------------------

    /**
     * @brief Send a command to the Tello drone.
     * @param command The command string to send.
     * @return Status of the command execution.
     */
    TelloStatus sendCommand(const char *command);

    /**
     * @brief Get the last response received from the Tello drone.
     * @return The last response string.
     */
    String getLastResponse() const;

    /**
     * @brief Get the last error message.
     * @return The last error message string.
     */
    String getLastErrorMessage() const;
    
    /**
     * @brief Clear the last error message and response.
     */
    void clearLastError();

    // ----------------------------------
    // Timeouts and Timing Constants
    // ----------------------------------

    static const unsigned long CONNECTION_TIMEOUT = 1000; // 1 seconds (for connection)
    static const unsigned long RESPONSE_TIMEOUT = 10000;    // 10 seconds (for command responses)
    static const unsigned long TIME_BTW_COMMANDS = 500;    // 500 ms between commands
    static const unsigned long TIME_BTW_RC_CONTROL_COMMANDS = 50; // 50 ms between RC control updates

    // ----------------------------------
    // Flight Control
    // ----------------------------------

    /**
     * @brief Command the Tello to take off.
     * @return Status of the takeoff command.
     */
    TelloStatus takeoff();

    /**
     * @brief Command the Tello to land.
     * @return Status of the land command.
     */
    TelloStatus land();

    /**
     * @brief Check if the Tello is currently flying.
     * @return True if flying, false otherwise.
     */
    bool isFlying() const { return is_flying; }

    // ----------------------------------
    // Movement Commands
    // ----------------------------------

    /**
     * @brief Move the Tello up.
     * @param cm Distance to move up in centimeters (20-500).
     * @return Status of the command.
     */
    TelloStatus move_up(int cm);

    /**
     * @brief Move the Tello down.
     * @param cm Distance to move down in centimeters (20-500).
     * @return Status of the command.
     */
    TelloStatus move_down(int cm);

    /**
     * @brief Move the Tello left.
     * @param cm Distance to move left in centimeters (20-500).
     * @return Status of the command.
     */
    TelloStatus move_left(int cm);

    /**
     * @brief Move the Tello right.
     * @param cm Distance to move right in centimeters (20-500).
     * @return Status of the command.
     */
    TelloStatus move_right(int cm);

    /**
     * @brief Move the Tello forward.
     * @param cm Distance to move forward in centimeters (20-500).
     * @return Status of the command.
     */
    TelloStatus move_forward(int cm);

    /**
     * @brief Move the Tello back.
     * @param cm Distance to move back in centimeters (20-500).
     * @return Status of the command.
     */
    TelloStatus move_back(int cm);

    /**
     * @brief Rotate the Tello clockwise.
     * @param degrees Degrees to rotate (1-360).
     * @return Status of the command.
     */
    TelloStatus rotate_clockwise(int degrees);

    /**
     * @brief Rotate the Tello counter-clockwise.
     * @param degrees Degrees to rotate (1-360).
     * @return Status of the command.
     */
    TelloStatus rotate_counter_clockwise(int degrees);

    // ----------------------------------
    // Flip Commands
    // ----------------------------------

    /**
     * @brief Flip the Tello to the left.
     * @return Status of the command.
     */
    TelloStatus flip_left();

    /**
     * @brief Flip the Tello to the right.
     * @return Status of the command.
     */
    TelloStatus flip_right();

    /**
     * @brief Flip the Tello forward.
     * @return Status of the command.
     */
    TelloStatus flip_forward();

    /**
     * @brief Flip the Tello backward.
     * @return Status of the command.
     */
    TelloStatus flip_back();

    // ----------------------------------
    // Motor Control
    // ----------------------------------

    /**
     * @brief Turn the Tello motors on (for cooling).
     * @return Status of the command.
     */
    TelloStatus turn_motor_on();

    /**
     * @brief Turn the Tello motors off.
     * @return Status of the command.
     */
    TelloStatus turn_motor_off();

    // ----------------------------------
    // Advanced Movement
    // ----------------------------------

    /**
     * @brief Move the Tello to a specific coordinate with a given speed.
     * @param x X-coordinate (-500 to 500 cm).
     * @param y Y-coordinate (-500 to 500 cm).
     * @param z Z-coordinate (-500 to 500 cm).
     * @param speed Speed (10-100 cm/s).
     * @return Status of the command.
     */
    TelloStatus go_xyz_speed(int x, int y, int z, int speed);

    /**
     * @brief Move the Tello along a curve with a given speed.
     * @param x1 First point X-coordinate (-500 to 500 cm).
     * @param y1 First point Y-coordinate (-500 to 500 cm).
     * @param z1 First point Z-coordinate (-500 to 500 cm).
     * @param x2 Second point X-coordinate (-500 to 500 cm).
     * @param y2 Second point Y-coordinate (-500 to 500 cm).
     * @param z2 Second point Z-coordinate (-500 to 500 cm).
     * @param speed Speed (10-60 cm/s).
     * @return Status of the command.
     */
    TelloStatus curve_xyz_speed(int x1, int y1, int z1, int x2, int y2, int z2, int speed);

    /**
     * @brief Set the Tello's speed.
     * @param speed Speed (10-100 cm/s).
     * @return Status of the command.
     */
    TelloStatus set_speed(int speed);

    /**
     * @brief Send RC control commands.
     * @param left_right_velocity Left/right velocity (-100 to 100).
     * @param forward_backward_velocity Forward/backward velocity (-100 to 100).
     * @param up_down_velocity Up/down velocity (-100 to 100).
     * @param yaw_velocity Yaw velocity (-100 to 100).
     * @note This command must be sent frequently (e.g., every 50ms) to maintain smooth control.
     */
    void send_rc_control(int left_right_velocity, int forward_backward_velocity,
                         int up_down_velocity, int yaw_velocity);

    // ----------------------------------
    // Query Commands
    // ----------------------------------

    /**
     * @brief Query the Tello's current speed setting.
     * @return Current speed setting (cm/s).
     */
    int query_speed() const;

    /**
     * @brief Query the Tello's current battery percentage.
     * @return Current battery percentage.
     */
    int query_battery() const;

    /**
     * @brief Query the Tello's current flight time.
     * @return Current flight time (seconds).
     */
    int query_time() const;

    /**
     * @brief Query the Tello's current height.
     * @return Current height (dm).
     */
    int query_height() const;

    /**
     * @brief Query the Tello's current temperature.
     * @return Current temperature (average if range).
     */
    int query_temp() const;

    /**
     * @brief Query the Tello's attitude (pitch, roll, yaw).
     * @return Current attitude as a string (e.g., "pitch:0;roll:0;yaw:0").
     */
    String query_attitude() const;

    /**
     * @brief Query the Tello's barometer reading.
     * @return Barometer reading (m).
     */
    float query_baro() const;

    /**
     * @brief Query the Tello's time-of-flight distance.
     * @return Time-of-flight distance (cm).
     */
    float query_tof() const;

    /**
     * @brief Query the Tello's WiFi SNR.
     * @return WiFi SNR as a string (e.g., "snr:90").
     */
    String query_wifi() const;

    /**
     * @brief Query the Tello's SDK version.
     * @return SDK version as a string (e.g., "SDK:2.0").
     */
    String query_sdk() const;

    /**
     * @brief Query the Tello's serial number.
     * @return Serial number as a string.
     */
    String query_sn() const;

    // ----------------------------------
    // Direct State Getters
    // ----------------------------------

    /**
     * @brief Get the Tello's current height (faster than query_height).
     * @return Current height in cm.
     */
    int get_height() const;

    /**
     * @brief Get the Tello's current battery percentage (faster than query_battery).
     * @return Current battery percentage.
     */
    int get_battery() const;

    /**
     * @brief Get the Tello's current flight time (faster than query_time).
     * @return Current flight time in seconds.
     */
    int get_flight_time() const;

    /**
     * @brief Get the Tello's current temperature (faster than query_temp).
     * @return Current temperature in Celsius.
     */
    float get_temperature() const;

    /**
     * @brief Get the Tello's current barometer reading (faster than query_baro).
     * @return Current barometer reading in meters.
     */
    float get_barometer() const;

    /**
     * @brief Get the Tello's current TOF distance (faster than query_tof).
     * @return Current TOF distance in cm.
     */
    float get_tof() const;

    /**
     * @brief Get the Tello's current pitch.
     * @return Current pitch in degrees.
     */
    int get_pitch() const;

    /**
     * @brief Get the Tello's current roll.
     * @return Current roll in degrees.
     */
    int get_roll() const;

    /**
     * @brief Get the Tello's current yaw.
     * @return Current yaw in degrees.
     */
    int get_yaw() const;

    /**
     * @brief Get the Tello's current x-axis speed.
     * @return Current x-axis speed in cm/s.
     */
    int get_speed_x() const;

    /**
     * @brief Get the Tello's current y-axis speed.
     * @return Current y-axis speed in cm/s.
     */
    int get_speed_y() const;

    /**
     * @brief Get the Tello's current z-axis speed.
     * @return Current z-axis speed in cm/s.
     */
    int get_speed_z() const;

    /**
     * @brief Get the Tello's current x-axis acceleration.
     * @return Current x-axis acceleration in cm/s^2.
     */
    float get_acceleration_x() const;

    /**
     * @brief Get the Tello's current y-axis acceleration.
     * @return Current y-axis acceleration in cm/s^2.
     */
    float get_acceleration_y() const;

    /**
     * @brief Get the Tello's current z-axis acceleration.
     * @return Current z-axis acceleration in cm/s^2.
     */
    float get_acceleration_z() const;

    // ----------------------------------
    // Video Configuration
    // ----------------------------------

    /**
     * @brief Set the video bitrate.
     * @param bitrate The desired bitrate.
     * @return Status of the command.
     */
    TelloStatus set_video_bitrate(VideoBitrate bitrate);

    /**
     * @brief Set the video resolution.
     * @param resolution The desired resolution.
     * @return Status of the command.
     */
    TelloStatus set_video_resolution(VideoResolution resolution);

    /**
     * @brief Set the video FPS.
     * @param fps The desired FPS.
     * @return Status of the command.
     */
    TelloStatus set_video_fps(VideoFPS fps);

    // ----------------------------------
    // Video Streaming
    // ----------------------------------

    /**
     * @brief Turn on video streaming.
     * @return Status of the command.
     */
    TelloStatus streamon();

    /**
     * @brief Turn off video streaming.
     * @return Status of the command.
     */
    TelloStatus streamoff();

    /**
     * @brief Callback function type for video data.
     * @param data Pointer to the video data buffer.
     * @param length Length of the video data in bytes.
     */
    typedef void (*VideoDataCallback)(const uint8_t *data, size_t length);

    /**
     * @brief Set the video data callback function.
     * @param callback The callback function to be called when video data is received.
     */
    void setVideoDataCallback(VideoDataCallback callback);
    
    /**
     * @brief Start the video stream.
     * This function sends the `streamon` command and starts the video receiving task.
     */
    void startVideoStream();

    // ----------------------------------
    // Error Callback
    // ----------------------------------

    /**
     * @brief Error callback function type.
     * @param command The command that was executed and resulted in an error.
     * @param errorMessage The error message returned by the drone or describing internal error.
     */
    typedef void (*ErrorCallback)(const char* command, const char* errorMessage);

    /**
     * @brief Set the error callback function.
     * @param callback The callback function to be called when a command returns an error.
     */
    void setErrorCallback(ErrorCallback callback);

    // ----------------------------------
    // Mission Pad
    // ----------------------------------

    /**
     * @brief Enable mission pad detection.
     * @return Status of the command.
     */
    TelloStatus enable_mission_pads();

    /**
     * @brief Disable mission pad detection.
     * @return Status of the command.
     */
    TelloStatus disable_mission_pads();

    /**
     * @brief Set the mission pad detection direction.
     * @param direction 0 for both downward and forward detection, 1 for downward only, 2 for forward only.
     * @return Status of the command.
     */
    TelloStatus set_mission_pad_detection_direction(int direction);

    // ----------------------------------
    // Network Configuration
    // ----------------------------------

    /**
     * @brief Set the WiFi credentials on the Tello drone (for station mode).
     * @param ssid The SSID of the network to connect to.
     * @param password The password of the network.
     * @return Status of the command.
     */
    TelloStatus set_wifi_credentials(const char *ssid, const char *password);

    /**
     * @brief Connect the Tello to a WiFi network (station mode).
     * @param ssid The SSID of the network to connect to.
     * @param password The password of the network.
     * @return Status of the command.
     */
    TelloStatus connect_to_wifi(const char *ssid, const char *password);

    // ----------------------------------
    // Camera Control
    // ----------------------------------

    /**
     * @brief Set the video direction (camera selection).
     * @param direction The camera direction (FORWARD or DOWNWARD).
     * @return Status of the command.
     */
    TelloStatus set_video_direction(CameraDirection direction);

    // ----------------------------------
    // Expansion Board
    // ----------------------------------

    /**
     * @brief Send a command to the Tello expansion board.
     * @param cmd The command string to send.
     * @return Status of the command.
     */
    TelloStatus send_expansion_command(const char *cmd);

    // ----------------------------------
    // Utility Functions
    // ----------------------------------

    /**
     * @brief Enable or disable the printing of raw state messages to the serial monitor.
     * @param enable True to enable, false to disable.
     */
    void enableRawStateMessages(bool enable = true);

    /**
     * @brief Emergency stop - immediately stops all motors.
     * Use with caution! The drone will fall from its current position.
     */
    void emergency();

private:
    // ----------------------------------
    // Internal Helper Functions
    // ----------------------------------

    TelloStatus _sendCommand(const char *command, CommandType type) const; // Handles both action and query commands internally
    String sendQueryCommand(const char *command) const;
    String waitForResponse(unsigned long timeout_ms) const;
    String readResponse() const; // Helper function for reading responses
    void waitBetweenCommands() const;
    int clamp100(int value) const;

    // ----------------------------------
    // Network and Communication
    // ----------------------------------

    bool connected = false;          // Flag to indicate if connected to Tello
    WiFiUDP udp;            // UDP socket for commands
    WiFiUDP state_udp;      // UDP socket for state data
    WiFiUDP video_udp;      // UDP socket for video stream
    IPAddress telloAddr;    // Tello IP address (default: 192.168.10.1)
    uint16_t telloPort;     // Tello command port (default: 8889)
    uint16_t localPort;     // Local port for command responses
    uint16_t statePort;     // Local port for state data
    uint16_t videoPort;     // Local port for video stream
    mutable String lastResponse;       // Last response received from Tello
    String lastErrorMessage;           // Last error message

    // ----------------------------------
    // Response Handling
    // ----------------------------------

    TaskHandle_t receiveTaskHandle;    // Task handle for the response receive task
    static const size_t MAX_RESPONSES = 10; // Maximum number of responses to store in the buffer
    mutable char responses[MAX_RESPONSES][MAX_RESPONSE_LENGTH]; // Fixed size char buffers for responses
    mutable volatile int responseHead; // Index for the next response to be added
    mutable volatile int responseTail; // Index for the next response to be read
    mutable portMUX_TYPE responseMux;  // Mutex for thread safety when accessing the response buffer

    // ----------------------------------
    // Task Management
    // ----------------------------------

    static void receiveTask(void *parameter); // Task function for receiving command responses and state data
    void startReceiveTask();
    void stopReceiveTask();
    bool responseTaskRunning;           // Flag to indicate if the response task is running

    // ----------------------------------
    // Flight Status and Timing
    // ----------------------------------

    bool is_flying;                     // Flag to indicate if the drone is currently flying
    mutable unsigned long last_received_command_timestamp; // Timestamp of the last received command
    mutable unsigned long last_rc_control_timestamp;      // Timestamp of the last RC control command

    // ----------------------------------
    // Error Callback
    // ----------------------------------
    
    ErrorCallback _errorCallback;

    // ----------------------------------
    // Video Streaming
    // ----------------------------------

    bool stream_on;                     // Flag to indicate if video streaming is enabled
    TaskHandle_t videoReceiveTaskHandle;// Task handle for the video receive task
    bool videoTaskRunning;              // Flag to indicate if the video task is running
    VideoDataCallback videoDataCallback;// Callback function for video data
    SemaphoreHandle_t videoTaskSemaphore; // Semaphore for synchronizing video task

    // ----------------------------------
    // Video Task Functions
    // ----------------------------------

    void startVideoReceiveTask();
    void stopVideoReceiveTask();
    static void videoReceiveTask(void *parameter);

    // ----------------------------------
    // State Variables
    // ----------------------------------

    struct TelloState {
        int height = 0;         // cm
        int battery = 0;        // %
        int flight_time = 0;    // seconds
        float temperature = 0;  // C
        float barometer = 0;    // meters
        float tof = 0;          // cm
        int pitch = 0;          // degrees
        int roll = 0;           // degrees
        int yaw = 0;           // degrees
        int speed_x = 0;        // cm/s
        int speed_y = 0;        // cm/s
        int speed_z = 0;        // cm/s
        float accel_x = 0;      // cm/s^2
        float accel_y = 0;      // cm/s^2
        float accel_z = 0;      // cm/s^2
        int temperature_low = 0; // Lower bound of temperature range
        int temperature_high = 0; // Upper bound of temperature range
    } state;

    // ----------------------------------
    // State Parsing
    // ----------------------------------

    void parse_state(const char *state_data) const;
    static const char VALUE_DELIMITER = ':'; // Delimiter between key and value in state data

    // ----------------------------------
    // State Field Mappings
    // ----------------------------------

    struct StateFieldMapping {
        const char *name;
        void (TelloESP32::*setter)(const char*) const; // Function pointer to the corresponding setter
    };

    // ----------------------------------
    // State Setters
    // ----------------------------------

    static const StateFieldMapping STATE_MAPPINGS[];
    void set_height(const char *value) const;
    void set_battery(const char *value) const;
    void set_flight_time(const char *value) const;
    void set_temperature_low(const char *value) const;
    void set_temperature_high(const char *value) const;
    void set_barometer(const char *value) const;
    void set_tof(const char *value) const;
    void set_pitch(const char *value) const;
    void set_roll(const char *value) const;
    void set_yaw(const char *value) const;
    void set_speed_x(const char *value) const;
    void set_speed_y(const char *value) const;
    void set_speed_z(const char *value) const;
    void set_acceleration_x(const char *value) const;
    void set_acceleration_y(const char *value) const;
    void set_acceleration_z(const char *value) const;

    // ----------------------------------
    // String Conversion Helpers
    // ----------------------------------

    static int stringToInt(const char *str);
    static float stringToFloat(const char *str);
};

} // namespace TelloControl

#endif // TELLOESP32_H