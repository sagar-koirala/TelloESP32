// TelloESP32.h
#ifndef TELLOESP32_H
#define TELLOESP32_H

// State message debug control
#ifndef print_Raw_State
#define print_Raw_State 0 // Set to 1 to enable state message debug output
#endif

// Debug logging control
#ifndef TELLO_DEBUG
#define TELLO_DEBUG 1  // Set to 1 to enable debug output
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

#include <WiFi.h>
#include <WiFiUdp.h>
#include <string.h> 
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Define constants for Tello ports
#define TELLO_CMD_PORT  8889
#define TELLO_STATE_PORT 8890
#define TELLO_VIDEO_PORT 11111

namespace TelloControl {

// --- Enums ---
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
class TelloESP32 {
public:
    TelloESP32();
    ~TelloESP32();

    // Connection and Initialization
    TelloStatus connect(const char *ssid, const char *password, unsigned long timeout_ms = 10000);
    void disconnect();
    bool isConnected() const { return connected; } // Check if connected (made const)

    // --- Command Sending ---
    TelloStatus sendCommand(const char *command);             // Send a command
    String getLastResponse() const;                           // Get the last response (made const)

    // --- Timeouts and Timing Constants ---
    static const unsigned long CONNECTION_TIMEOUT = 500;     // 10 seconds (for connection)
    static const unsigned long RESPONSE_TIMEOUT = 7000;          // 7 seconds (for command responses)
    static const unsigned long TIME_BTW_COMMANDS = 500;          // 500 ms between commands
    static const unsigned long TIME_BTW_RC_CONTROL_COMMANDS = 50; // 50 ms between RC control updates

    // --- Flight Control ---
    TelloStatus takeoff();
    TelloStatus land();
    bool isFlying() const { return is_flying; }                 // Check if the drone is flying (made const)

    // --- Movement Commands (with range checks) ---
    TelloStatus move_up(int cm);                              // 20-500 cm
    TelloStatus move_down(int cm);                            // 20-500 cm
    TelloStatus move_left(int cm);                            // 20-500 cm
    TelloStatus move_right(int cm);                           // 20-500 cm
    TelloStatus move_forward(int cm);                         // 20-500 cm
    TelloStatus move_back(int cm);                            // 20-500 cm
    TelloStatus rotate_clockwise(int degrees);               // 1-360 degrees
    TelloStatus rotate_counter_clockwise(int degrees);       // 1-360 degrees

    // --- Flip Commands ---
    TelloStatus flip_left();
    TelloStatus flip_right();
    TelloStatus flip_forward();
    TelloStatus flip_back();

    // --- Motor Control ---
    TelloStatus turn_motor_on();
    TelloStatus turn_motor_off();

    // --- Advanced Movement ---
    TelloStatus go_xyz_speed(int x, int y, int z, int speed); // x, y, z: -500 to 500, speed: 10-100
    TelloStatus curve_xyz_speed(int x1, int y1, int z1, int x2, int y2, int z2, int speed); // x1, y1, z1, x2, y2, z2: -500 to 500, speed: 10-60
    TelloStatus set_speed(int speed);                           // speed: 10-100
    void send_rc_control(int left_right_velocity, int forward_backward_velocity,
                         int up_down_velocity, int yaw_velocity); // -100 to 100 for each

    // --- Query Commands ---
    int query_speed() const;               // Get current speed setting
    int query_battery() const;             // Get current battery percentage
    int query_time() const;                // Get current flight time
    int query_height() const;              // Get current height (dm)
    int query_temp() const;                // Get current temperature (average if range)
    float query_attitude() const;         // Get current attitude (pitch, roll, yaw)
    float query_baro() const;              // Get barometer reading
    float query_tof() const;               // Get time-of-flight distance
    String query_wifi() const;             // Get WiFi SNR
    String query_sdk() const;              // Get SDK version
    String query_sn() const;               // Get serial number

    // --- Direct State Getters (faster than queries) ---
    int get_height() const;                // Current height in cm
    int get_battery() const;               // Current battery percentage
    int get_flight_time() const;           // Current flight time in seconds
    float get_temperature() const;         // Current temperature in Celsius
    float get_barometer() const;           // Current barometer reading in meters
    float get_tof() const;                 // Current TOF distance in cm
    int get_pitch() const;                 // Current pitch in degrees
    int get_roll() const;                  // Current roll in degrees
    int get_yaw() const;                   // Current yaw in degrees
    int get_speed_x() const;               // Current x-axis speed in cm/s
    int get_speed_y() const;               // Current y-axis speed in cm/s
    int get_speed_z() const;               // Current z-axis speed in cm/s
    float get_acceleration_x() const;      // Current x-axis acceleration in cm/s^2
    float get_acceleration_y() const;      // Current y-axis acceleration in cm/s^2
    float get_acceleration_z() const;      // Current z-axis acceleration in cm/s^2

    // --- Video Configuration ---
    TelloStatus set_video_bitrate(VideoBitrate bitrate);
    TelloStatus set_video_resolution(VideoResolution resolution);
    TelloStatus set_video_fps(VideoFPS fps);

    // --- Video Streaming ---
    TelloStatus streamon();
    TelloStatus streamoff();
    typedef void (*VideoDataCallback)(const uint8_t *data, size_t length); // Callback function type for video data
    void setVideoDataCallback(VideoDataCallback callback);                 // Set the video data callback function

    // --- Mission Pad ---
    TelloStatus enable_mission_pads();
    TelloStatus disable_mission_pads();
    TelloStatus set_mission_pad_detection_direction(int direction); // 0: both, 1: downward, 2: forward

    // --- Network Configuration ---
    TelloStatus set_wifi_credentials(const char *ssid, const char *password);
    TelloStatus connect_to_wifi(const char *ssid, const char *password);

    // --- Camera Control ---
    TelloStatus set_video_direction(CameraDirection direction);

    // --- Expansion Board ---
    TelloStatus send_expansion_command(const char *cmd);

    void enableRawStateMessages(bool enable = true); // Enable/disable raw state message printing

    // Start recording video and save to SD card
    bool startVideoRecording(const char* filename);

    // Start video stream without recording
    void startVideoStream();

private:
    // --- Internal Helper Functions ---
    TelloStatus _sendCommand(const char *command, CommandType type) const; // Handles both action and query commands internally
    String sendQueryCommand(const char *command) const;             // Sends a query command
    String waitForResponse(unsigned long timeout_ms) const;  // Make const
    String readResponse() const; // Helper function for read

    // --- Network and Communication ---
    bool connected;             // Flag to indicate if connected to Tello
    WiFiUDP udp;                 // UDP socket for commands
    WiFiUDP state_udp;           // UDP socket for state data
    WiFiUDP video_udp;           // UDP socket for video stream
    IPAddress telloAddr;         // Tello IP address (default: 192.168.10.1)
    uint16_t telloPort;          // Tello command port (default: 8889)
    uint16_t localPort;          // Local port for command responses
    uint16_t statePort;          // Local port for state data
    uint16_t videoPort;          // Local port for video stream
    mutable String lastResponse; // Last response received from Tello (mutable for const methods)

    // --- Response Handling ---
    TaskHandle_t receiveTaskHandle;         // Task handle for the response receive task
    static const size_t MAX_RESPONSES = 10; // Maximum number of responses to store in the buffer
    static const size_t MAX_RESPONSE_LENGTH = 128; // Increased buffer size to 128
    mutable char responses[MAX_RESPONSES][MAX_RESPONSE_LENGTH]; // Fixed size char buffers for responses
    mutable volatile int responseHead;              // Index for the next response to be added
    mutable volatile int responseTail;              // Index for the next response to be read
    mutable portMUX_TYPE responseMux;               // Mutex for thread safety when accessing the response buffer

    // --- Task Management ---
    static void receiveTask(void *parameter); // Task function for receiving command responses and state data
    void startReceiveTask();                 // Starts the receive task
    void stopReceiveTask();                  // Stops the receive task
    bool responseTaskRunning;                // Flag to indicate if the response task is running

    // --- Flight Status and Timing ---
    bool is_flying;                          // Flag to indicate if the drone is currently flying
    mutable unsigned long last_received_command_timestamp; // Timestamp of the last received command
    mutable unsigned long last_rc_control_timestamp;      // Timestamp of the last RC control command
    
    // --- Video Streaming ---
    bool stream_on;                             // Flag to indicate if video streaming is enabled
    TaskHandle_t videoReceiveTaskHandle;        // Task handle for the video receive task
    bool videoTaskRunning;                     // Flag to indicate if the video task is running
    VideoDataCallback videoDataCallback;        // Callback function for video data
    SemaphoreHandle_t videoTaskSemaphore;       // Semaphore for synchronizing video task

    // --- Video Task Functions ---
    void startVideoReceiveTask();                 // Starts the video receive task
    void stopVideoReceiveTask();                  // Stops the video receive task
    static void videoReceiveTask(void *parameter); // Task function for receiving video data

    // --- Internal Helpers ---
    void waitBetweenCommands() const;            // Waits for the required time between commands
    int clamp100(int value) const;                // Clamps a value between -100 and 100

    // --- State Variables ---
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
        int temperature_low;
        int temperature_high;
    } state;

    // --- State Parsing ---
    void parse_state(const char *state_data);    // Parses the state data string
    static const char VALUE_DELIMITER = ':';     // Delimiter between key and value in state data

    // --- State Field Mappings ---
    struct StateFieldMapping {
        const char *name;
        void (TelloESP32::*setter)(const char*); // Function pointer to the corresponding setter
    };
    static const StateFieldMapping STATE_MAPPINGS[]; // Array of mappings for state data fields

    // --- State Setters ---
    void set_height(const char *value);
    void set_battery(const char *value);
    void set_flight_time(const char *value);
    void set_temperature_low(const char *value);
    void set_temperature_high(const char *value);
    void set_barometer(const char *value);
    void set_tof(const char *value);
    void set_pitch(const char *value);
    void set_roll(const char *value);
    void set_yaw(const char *value);
    void set_speed_x(const char *value);
    void set_speed_y(const char *value);
    void set_speed_z(const char *value);
    void set_acceleration_x(const char *value);
    void set_acceleration_y(const char *value);
    void set_acceleration_z(const char *value);

    // --- String Conversion Helpers ---
    static int stringToInt(const char *str);
    static float stringToFloat(const char *str);
};

} // namespace TelloControl

#endif // TELLOESP32_H