/*
  TelloESP32.cpp - Implementation of the TelloESP32 library
  See header file for documentation
*/

// TelloESP32.cpp
#include "TelloESP32.h"
#include <Arduino.h>
#include <string.h> // For string functions like strtok
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <WiFi.h>
#include <SD_MMC.h>  // Use SD_MMC library

namespace TelloControl {

// --- State Field Mappings ---
const TelloESP32::StateFieldMapping TelloESP32::STATE_MAPPINGS[] = {
    {"h", &TelloESP32::set_height},
    {"bat", &TelloESP32::set_battery},
    {"time", &TelloESP32::set_flight_time},
    {"baro", &TelloESP32::set_barometer},
    {"tof", &TelloESP32::set_tof},
    {"pitch", &TelloESP32::set_pitch},
    {"roll", &TelloESP32::set_roll},
    {"yaw", &TelloESP32::set_yaw},
    {"vgx", &TelloESP32::set_speed_x},
    {"vgy", &TelloESP32::set_speed_y},
    {"vgz", &TelloESP32::set_speed_z},
    {"agx", &TelloESP32::set_acceleration_x},
    {"agy", &TelloESP32::set_acceleration_y},
    {"agz", &TelloESP32::set_acceleration_z},
    {"templ", &TelloESP32::set_temperature_low},
    {"temph", &TelloESP32::set_temperature_high}
};

// --- Constructor Implementation ---
TelloESP32::TelloESP32()
    : telloAddr(192, 168, 10, 1),
      telloPort(TELLO_CMD_PORT),
      localPort(9000),
      statePort(TELLO_STATE_PORT),
      videoPort(TELLO_VIDEO_PORT),
      connected(false),
      receiveTaskHandle(nullptr),
      responseTaskRunning(false),
      responseHead(0),
      responseTail(0),
      is_flying(false),
      last_received_command_timestamp(0),
      last_rc_control_timestamp(0),
      stream_on(false),
      videoReceiveTaskHandle(nullptr),
      videoTaskRunning(false),
      videoDataCallback(nullptr)
{
    // Initialize mutex for thread-safe response buffer access
    responseMux = portMUX_INITIALIZER_UNLOCKED;
    // Initialize semaphore for video task synchronization
    videoTaskSemaphore = xSemaphoreCreateBinary();
}

// --- Start Receive Task ---
void TelloESP32::startReceiveTask() {
    if (!responseTaskRunning && receiveTaskHandle == nullptr) {
        responseTaskRunning = true;
        // Create the response receive task (pinned to core 0)
        xTaskCreatePinnedToCore(
            receiveTask,       // Task function
            "TelloReceive",    // Task name
            4096,              // Stack size (4KB) - adjust as necessary
            this,              // Task parameter (this pointer)
            2,                 // Task priority
            &receiveTaskHandle, // Task handle
            0                  // Core ID (0 or 1) - pinned to core 0
        );
    }
}

// --- Stop Receive Task ---
void TelloESP32::stopReceiveTask() {
    if (responseTaskRunning && receiveTaskHandle != nullptr) {
        responseTaskRunning = false; // Signal the task to stop
        // Wait for the task to finish
        while (receiveTaskHandle != nullptr) {
            vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for the task to exit
        }
        // No need to call vTaskDelete here since the task deletes itself
    }
}

// --- Receive Task ---
void TelloESP32::receiveTask(void *parameter) {
    TelloESP32 *tello = static_cast<TelloESP32 *>(parameter);
    char cmd_buffer[TelloESP32::MAX_RESPONSE_LENGTH]; // Buffer for command responses
    char state_buffer[256]; // Buffer for state data
    unsigned long last_state_time = millis(); // Track the time of the last received state

    while (tello->responseTaskRunning) {
        // --- Check for command responses ---
        int cmd_packetSize = tello->udp.parsePacket();
        if (cmd_packetSize > 0 && cmd_packetSize <= TelloESP32::MAX_RESPONSE_LENGTH) {
            int len = tello->udp.read(cmd_buffer, sizeof(cmd_buffer) - 1);
            if (len > 0) {
                cmd_buffer[len] = '\0'; // Null-terminate the string

                // --- Store response in the circular buffer (thread-safe) ---
                portENTER_CRITICAL(&tello->responseMux);
                int nextHead = (tello->responseHead + 1) % MAX_RESPONSES;
                if (nextHead != tello->responseTail) {
                    strncpy(tello->responses[tello->responseHead], cmd_buffer, sizeof(tello->responses[tello->responseHead]));
                    tello->responses[tello->responseHead][sizeof(tello->responses[tello->responseHead]) - 1] = '\0'; // Ensure null termination
                    tello->responseHead = nextHead;
                }
                portEXIT_CRITICAL(&tello->responseMux);
            }
        } else if (cmd_packetSize > TelloESP32::MAX_RESPONSE_LENGTH) {
            // Handle the case where the packet size exceeds the maximum response length
            Serial.println("Error: Received command response exceeds maximum length.");
        }

        // --- Check for state data ---
        int state_packetSize = tello->state_udp.parsePacket();
        if (state_packetSize > 0) {
            int len = tello->state_udp.read(state_buffer, sizeof(state_buffer) - 1);
            if (len > 0) {
                state_buffer[len] = '\0'; // Null-terminate the string
                tello->parse_state(state_buffer); // Parse the state data
                last_state_time = millis(); // Update last received state time
            }
        }

        // --- Check for connection loss ---
        if (millis() - last_state_time > TelloESP32::CONNECTION_TIMEOUT) {
            tello->connected = false;
            memset(&tello->state, 0, sizeof(tello->state));
            TELLO_LOGLN("Connection lost. State variables reset.");
        }

        vTaskDelay(10 / portTICK_PERIOD_MS); // Delay for 10ms
    }
    tello->receiveTaskHandle = nullptr; // Indicate that the task has exited
    vTaskDelete(NULL); // Self-delete the task
}

// --- Connection Management ---
TelloStatus TelloESP32::connect(const char *ssid, const char *password, unsigned long timeout_ms) {
    unsigned long startTime = millis();
    WiFi.begin(ssid, password);
    TELLO_LOG("Connecting to Tello");

    // --- Wait for WiFi connection ---
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - startTime > timeout_ms) {
            TELLO_LOGLN(" Connection timed out!");
            WiFi.disconnect(true);
            return TelloStatus::Timeout;
        }
        delay(350);
        TELLO_LOG(".");
    }

    // --- Initialize UDP ---
    if (!udp.begin(localPort)) {
        TELLO_LOGLN("Failed to initialize command UDP.");
        WiFi.disconnect(true);
        return TelloStatus::UdpError;
    }
    if (!state_udp.begin(statePort)) {
        TELLO_LOGLN("Failed to initialize state UDP.");
        udp.stop();
        WiFi.disconnect(true);
        return TelloStatus::UdpError;
    }

    delay(10000);       // Wait for Tello to boot up (important!)
    startReceiveTask(); // Start the receive task to listen for responses
    connected = true;
    TELLO_LOGLN(" Connected!");

    // --- Send command to enter SDK mode ---
    TelloStatus status = sendCommand("command");
    if (status == TelloStatus::OK) {
        return TelloStatus::OK;
    } else {
        TELLO_LOGLN("Failed to activate SDK mode");
        disconnect(); // Clean up resources
        return status;
    }
}

// --- Wait Between Commands ---
void TelloESP32::waitBetweenCommands() const {
    unsigned long current_time = millis();
    unsigned long diff = current_time - last_received_command_timestamp;
    if (diff < TIME_BTW_COMMANDS) {
        delay(TIME_BTW_COMMANDS - diff);
    }
}

// --- Internal Send Command (Action or Query) ---
TelloStatus TelloESP32::_sendCommand(const char *command, CommandType type) const {
    if (!connected) {
        lastResponse = (type == CommandType::Query) ? "" : "Not connected";
        return TelloStatus::NotConnected;
    }

    waitBetweenCommands(); // Wait for the required time between commands

    // --- Send the command ---
    TELLO_LOG("Sending ");
    TELLO_LOG(type == CommandType::Query ? "query: " : "command: ");
    TELLO_LOG(command);
    const_cast<TelloESP32*>(this)->udp.beginPacket(telloAddr, telloPort);
    const_cast<TelloESP32*>(this)->udp.write((const uint8_t *)command, strlen(command));
    const_cast<TelloESP32*>(this)->udp.endPacket();

    last_received_command_timestamp = millis(); // Update the timestamp of the last sent command

    // --- Wait for and handle the response (for action commands) ---
    if (type == CommandType::Action) {
        String response = waitForResponse(RESPONSE_TIMEOUT);
        if (response == "ok") {
            TELLO_LOG(" .... Response: ");
            TELLO_LOGLN(response);
            lastResponse = response;
            return TelloStatus::OK;
        } else if (response.length() > 0) {
            TELLO_LOG(" .... Response: ");
            TELLO_LOGLN(response);
            lastResponse = response;
            return TelloStatus::UnknownError;
        } else {
            TELLO_LOGLN("  !!! No response received.");
            lastResponse = "No response";
            return TelloStatus::NoResponse;
        }
    } else {
        // For query commands, the response is handled by sendQueryCommand()
        String response = waitForResponse(RESPONSE_TIMEOUT);
        lastResponse = response; 
        return TelloStatus::OK;
    }
}

// --- Send a Command ---
TelloStatus TelloESP32::sendCommand(const char *command) {
    return _sendCommand(command, CommandType::Action);
}

// --- Send a Query Command and Return the Response ---
String TelloESP32::sendQueryCommand(const char *command) const {
    if (_sendCommand(command, CommandType::Query) == TelloStatus::OK) {
        return lastResponse;
    }
    return ""; // Return an empty string on error
}

// --- Takeoff Command ---
TelloStatus TelloESP32::takeoff() {
    if (!connected) return TelloStatus::NotConnected;
    if (is_flying) {
        TELLO_LOGLN("Already flying");
        return TelloStatus::OK;
    }

    TelloStatus status = sendCommand("takeoff");
    if (status == TelloStatus::OK) is_flying = true;
    return status;
}

// --- Land Command ---
TelloStatus TelloESP32::land() {
    if (!connected) return TelloStatus::NotConnected;
    if (!is_flying) {
        TELLO_LOGLN("Already landed");
        return TelloStatus::OK;
    }

    TelloStatus status = sendCommand("land");
    if (status == TelloStatus::OK) is_flying = false;
    return status;
}

// --- Movement Commands with Range Checks ---
TelloStatus TelloESP32::move_up(int cm) {
    if (cm < 20 || cm > 500) return TelloStatus::InvalidParameter;
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "up %d", cm);
    return sendCommand(cmd);
}

TelloStatus TelloESP32::move_down(int cm) {
    if (cm < 20 || cm > 500) return TelloStatus::InvalidParameter;
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "down %d", cm);
    return sendCommand(cmd);
}

TelloStatus TelloESP32::move_left(int cm) {
    if (cm < 20 || cm > 500) return TelloStatus::InvalidParameter;
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "left %d", cm);
    return sendCommand(cmd);
}

TelloStatus TelloESP32::move_right(int cm) {
    if (cm < 20 || cm > 500) return TelloStatus::InvalidParameter;
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "right %d", cm);
    return sendCommand(cmd);
}

TelloStatus TelloESP32::move_forward(int cm) {
    if (cm < 20 || cm > 500) return TelloStatus::InvalidParameter;
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "forward %d", cm);
    return sendCommand(cmd);
}

TelloStatus TelloESP32::move_back(int cm) {
    if (cm < 20 || cm > 500) return TelloStatus::InvalidParameter;
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "back %d", cm);
    return sendCommand(cmd);
}

TelloStatus TelloESP32::rotate_clockwise(int degrees) {
    if (degrees < 1 || degrees > 360) return TelloStatus::InvalidParameter;
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "cw %d", degrees);
    return sendCommand(cmd);
}

TelloStatus TelloESP32::rotate_counter_clockwise(int degrees) {
    if (degrees < 1 || degrees > 360) return TelloStatus::InvalidParameter;
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "ccw %d", degrees);
    return sendCommand(cmd);
}

// --- Flip Commands ---
TelloStatus TelloESP32::flip_left() {
    return sendCommand("flip l");
}

TelloStatus TelloESP32::flip_right() {
    return sendCommand("flip r");
}

TelloStatus TelloESP32::flip_forward() {
    return sendCommand("flip f");
}

TelloStatus TelloESP32::flip_back() {
    return sendCommand("flip b");
}

// --- Disconnect from the Tello ---
void TelloESP32::disconnect() {
    if (connected) {
        if (is_flying) {
            land(); // Attempt to land before disconnecting
        }
        stopReceiveTask();
        stopVideoReceiveTask();

        udp.stop();
        state_udp.stop();
        video_udp.stop();
        connected = false;
        WiFi.disconnect(true);

        // Reset drone state variables
        memset(&state, 0, sizeof(state));
    }
}

// --- Wait for a Response from the Tello ---
String TelloESP32::waitForResponse(unsigned long timeout_ms) const {
    unsigned long startTime = millis();

    while (millis() - startTime < timeout_ms) {
        portENTER_CRITICAL(&responseMux);
        if (responseHead != responseTail) {
            String response = responses[responseTail];
            responseTail = (responseTail + 1) % MAX_RESPONSES;
            portEXIT_CRITICAL(&responseMux);
            return response;
        }
        portEXIT_CRITICAL(&responseMux);
        delay(10);
    }
    return ""; // Return empty string if no response within timeout
}

// --- Get the Last Response Received from the Tello (made const) ---
String TelloESP32::getLastResponse() const {
    return lastResponse;
}

// --- Destructor ---
TelloESP32::~TelloESP32() {
    if (stream_on) {
        streamoff();
    }
    disconnect();
    vSemaphoreDelete(videoTaskSemaphore);
}

// --- Turn Motor On ---
TelloStatus TelloESP32::turn_motor_on() {
    return sendCommand("motoron");
}

// --- Turn Motor Off ---
TelloStatus TelloESP32::turn_motor_off() {
    return sendCommand("motoroff");
}

// --- Go to XYZ with Speed ---
TelloStatus TelloESP32::go_xyz_speed(int x, int y, int z, int speed) {
    if (x < -500 || x > 500 || y < -500 || y > 500 || z < -500 || z > 500 ||
        speed < 10 || speed > 100) {
        return TelloStatus::InvalidParameter;
    }
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "go %d %d %d %d", x, y, z, speed);
    return sendCommand(cmd);
}

// --- Curve at XYZ with Speed ---
TelloStatus TelloESP32::curve_xyz_speed(int x1, int y1, int z1,
                                        int x2, int y2, int z2, int speed) {
    if (x1 < -500 || x1 > 500 || y1 < -500 || y1 > 500 || z1 < -500 || z1 > 500 ||
        x2 < -500 || x2 > 500 || y2 < -500 || y2 > 500 || z2 < -500 || z2 > 500 ||
        speed < 10 || speed > 60) {
        return TelloStatus::InvalidParameter;
    }
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "curve %d %d %d %d %d %d %d",
             x1, y1, z1, x2, y2, z2, speed);
    return sendCommand(cmd);
}

// --- Set Speed ---
TelloStatus TelloESP32::set_speed(int speed) {
    if (speed < 10 || speed > 100) {
        return TelloStatus::InvalidParameter;
    }
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "speed %d", speed);
    return sendCommand(cmd);
}

// --- Send RC Control ---
void TelloESP32::send_rc_control(int left_right_velocity,
                                 int forward_backward_velocity,
                                 int up_down_velocity,
                                 int yaw_velocity) {
    if (!connected) return;

    unsigned long current_time = millis();
    if (current_time - last_rc_control_timestamp > TIME_BTW_RC_CONTROL_COMMANDS) {
        char cmd[64];
        snprintf(cmd, sizeof(cmd), "rc %d %d %d %d",
                 clamp100(left_right_velocity),
                 clamp100(forward_backward_velocity),
                 clamp100(up_down_velocity),
                 clamp100(yaw_velocity));

        udp.beginPacket(telloAddr, telloPort);
        udp.write((const uint8_t *)cmd, strlen(cmd));
        udp.endPacket();

        last_rc_control_timestamp = current_time;
    }
}

// --- Query Functions ---
int TelloESP32::query_speed() const {
    return stringToInt(sendQueryCommand("speed?").c_str());
}

int TelloESP32::query_battery() const {
    return stringToInt(sendQueryCommand("battery?").c_str());
}

int TelloESP32::query_time() const {
    return stringToInt(sendQueryCommand("time?").c_str());
}

int TelloESP32::query_height() const {
    return stringToInt(sendQueryCommand("height?").c_str());
}

int TelloESP32::query_temp() const {
    String response = sendQueryCommand("temp?");
    if (response.indexOf('~') != -1) {
        int minTemp = stringToInt(response.substring(0, response.indexOf('~')).c_str());
        int maxTemp = stringToInt(response.substring(response.indexOf('~') + 1, response.length() - 1).c_str());
        return (minTemp + maxTemp) / 2;
    } else {
        return stringToInt(response.c_str());
    }
}

float TelloESP32::query_attitude() const {
    return stringToFloat(sendQueryCommand("attitude?").c_str());
}

float TelloESP32::query_baro() const {
    return stringToFloat(sendQueryCommand("baro?").c_str());
}

float TelloESP32::query_tof() const {
    return stringToFloat(sendQueryCommand("tof?").c_str());
}

String TelloESP32::query_wifi() const {
    return sendQueryCommand("wifi?");
}

String TelloESP32::query_sdk() const {
    return sendQueryCommand("sdk?");
}

String TelloESP32::query_sn() const {
    return sendQueryCommand("sn?");
}

// --- Direct State Getters (const methods) ---
int TelloESP32::get_height() const          { return state.height; }
int TelloESP32::get_battery() const         { return state.battery; }
int TelloESP32::get_flight_time() const     { return state.flight_time; }
float TelloESP32::get_temperature() const   { return (state.temperature_low + state.temperature_high) / 2.0f; }
float TelloESP32::get_barometer() const     { return state.barometer; }
float TelloESP32::get_tof() const           { return state.tof; }
int TelloESP32::get_pitch() const           { return state.pitch; }
int TelloESP32::get_roll() const            { return state.roll; }
int TelloESP32::get_yaw() const             { return state.yaw; }
int TelloESP32::get_speed_x() const         { return state.speed_x; }
int TelloESP32::get_speed_y() const         { return state.speed_y; }
int TelloESP32::get_speed_z() const         { return state.speed_z; }
float TelloESP32::get_acceleration_x() const { return state.accel_x; }
float TelloESP32::get_acceleration_y() const { return state.accel_y; }
float TelloESP32::get_acceleration_z() const { return state.accel_z; }

// --- State Parsing Function (using strtok for efficiency) ---
void TelloESP32::parse_state(const char *state_data) {
    // Print raw state message if enabled
    if (print_Raw_State) {
        TELLO_LOG("Raw State: ");
        TELLO_LOGLN(state_data);
    }

    char *data_copy = strdup(state_data); // Create a copy for strtok to modify
    char *token = strtok(data_copy, ";");

    while (token != NULL) {
        char *separator = strchr(token, VALUE_DELIMITER);
        if (separator != NULL) {
            *separator = '\0'; // Null-terminate the key
            const char *key = token;
            const char *value = separator + 1;

            // Find the corresponding state field and call its setter
            for (const auto &mapping : STATE_MAPPINGS) {
                if (strcmp(key, mapping.name) == 0) {
                    (this->*mapping.setter)(value);
                    break;
                }
            }
        }
        token = strtok(NULL, ";");
    }
    free(data_copy); // Free the allocated memory
}

// --- State Setters ---
void TelloESP32::set_height(const char *value)      { state.height = stringToInt(value); }
void TelloESP32::set_battery(const char *value)     { state.battery = stringToInt(value); }
void TelloESP32::set_flight_time(const char *value) { state.flight_time = stringToInt(value); }
void TelloESP32::set_barometer(const char *value)   { state.barometer = stringToFloat(value); }
void TelloESP32::set_tof(const char *value)         { state.tof = stringToFloat(value); }
void TelloESP32::set_pitch(const char *value)       { state.pitch = stringToInt(value); }
void TelloESP32::set_roll(const char *value)        { state.roll = stringToInt(value); }
void TelloESP32::set_yaw(const char *value)         { state.yaw = stringToInt(value); }
void TelloESP32::set_speed_x(const char *value)     { state.speed_x = stringToInt(value); }
void TelloESP32::set_speed_y(const char *value)     { state.speed_y = stringToInt(value); }
void TelloESP32::set_speed_z(const char *value)     { state.speed_z = stringToInt(value); }
void TelloESP32::set_acceleration_x(const char *value) { state.accel_x = stringToFloat(value); }
void TelloESP32::set_acceleration_y(const char *value) { state.accel_y = stringToFloat(value); }
void TelloESP32::set_acceleration_z(const char *value) { state.accel_z = stringToFloat(value); }
void TelloESP32::set_temperature_low(const char *value) { state.temperature_low = stringToInt(value); }
void TelloESP32::set_temperature_high(const char *value) { state.temperature_high = stringToInt(value); }

// --- String Conversion Helpers ---
int TelloESP32::stringToInt(const char *str) {
    if (str == NULL || *str == '\0') {
        return 0; 
    }
    return atoi(str);
}

float TelloESP32::stringToFloat(const char *str) {
    if (str == NULL || *str == '\0') {
        return 0.0f;
    }
    return atof(str);
}

// --- Clamp Value between -100 and 100 ---
int TelloESP32::clamp100(int value) const {
    return (value > 100) ? 100 : ((value < -100) ? -100 : value);
}

// --- Set Video Bitrate ---
TelloStatus TelloESP32::set_video_bitrate(VideoBitrate bitrate) {
    if (!connected) return TelloStatus::NotConnected;
    
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "setbitrate %d", static_cast<int>(bitrate));
    return sendCommand(cmd);
}

// --- Set Video Resolution ---
TelloStatus TelloESP32::set_video_resolution(VideoResolution resolution) {
    if (!connected) return TelloStatus::NotConnected;

    const char *resolution_str;
    switch (resolution) {
        case VideoResolution::RESOLUTION_720P: resolution_str = "high"; break;
        case VideoResolution::RESOLUTION_480P: resolution_str = "low"; break;
        default: return TelloStatus::InvalidParameter;
    }

    char cmd[32];
    snprintf(cmd, sizeof(cmd), "setresolution %s", resolution_str);
    return sendCommand(cmd);
}

// --- Set Video FPS ---
TelloStatus TelloESP32::set_video_fps(VideoFPS fps){
    if (!connected) return TelloStatus::NotConnected;

    const char *fps_str;
    switch (fps){
        case VideoFPS::FPS_5:  fps_str = "low"; break;
        case VideoFPS::FPS_15: fps_str = "middle"; break;
        case VideoFPS::FPS_30: fps_str = "high"; break;
        default: return TelloStatus::InvalidParameter;
    }

    char cmd[32];
    snprintf(cmd, sizeof(cmd), "setfps %s", fps_str);
    return sendCommand(cmd);
}

// --- Set Video Direction ---
TelloStatus TelloESP32::set_video_direction(CameraDirection direction) {
    if (!connected) return TelloStatus::NotConnected;
    
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "downvision %d", static_cast<int>(direction));
    return sendCommand(cmd);
}

// --- Stream On ---
TelloStatus TelloESP32::streamon(){
    if (!connected) return TelloStatus::NotConnected;

    TelloStatus status = sendCommand("streamon");
    if (status == TelloStatus::OK){
        stream_on = true;
        startVideoReceiveTask(); // Start video receive task
    }
    return status;
}

// --- Stream Off ---
TelloStatus TelloESP32::streamoff(){
    if (!connected) return TelloStatus::NotConnected;

    TelloStatus status = sendCommand("streamoff");
    if (status == TelloStatus::OK){
        stream_on = false;
        stopVideoReceiveTask(); // Stop video receive task
    }
    return status;
}

// --- Set Video Data Callback ---
void TelloESP32::setVideoDataCallback(VideoDataCallback callback){
    videoDataCallback = callback;
}

// --- Start Video Receive Task ---
void TelloESP32::startVideoReceiveTask(){
    if (!videoTaskRunning && videoReceiveTaskHandle == nullptr){
        videoTaskRunning = true;
        xSemaphoreGive(videoTaskSemaphore);  // Reset semaphore state

        // Begin listening on the video stream port
        if (!video_udp.begin(videoPort)){
            TELLO_LOGLN("Failed to initialize video UDP.");
            return;
        }

        xTaskCreate(
            videoReceiveTask,    // Task function
            "TelloVideoReceive", // Task name
            8192,                // Stack size (8KB) - adjust as necessary
            this,                // Task parameter (this pointer)
            2,                   // Task priority
            &videoReceiveTaskHandle // Task handle
        );
    }
}

// --- Stop Video Receive Task ---
void TelloESP32::stopVideoReceiveTask(){
    if (videoTaskRunning && videoReceiveTaskHandle != nullptr){
        videoTaskRunning = false; // Signal the task to stop

        // Wait for the task to finish (with timeout)
        if (xSemaphoreTake(videoTaskSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE){
            // Stop UDP after the task has finished
            video_udp.stop();

            // Now safe to delete the task
            vTaskDelete(videoReceiveTaskHandle);
            videoReceiveTaskHandle = nullptr;
        } else {
            // Emergency cleanup if the task doesn't respond
            TELLO_LOGLN("Warning: Video task did not respond to stop signal.");
            video_udp.stop();
            vTaskDelete(videoReceiveTaskHandle);
            videoReceiveTaskHandle = nullptr;
        }
    }
}

// --- Video Receive Task ---
void TelloESP32::videoReceiveTask(void *parameter){
    TelloESP32 *tello = static_cast<TelloESP32 *>(parameter);
    uint8_t buffer[2048]; // Buffer for video data (adjust size as needed)

    while (tello->videoTaskRunning){
        int packetSize = tello->video_udp.parsePacket();
        if (packetSize > 0){
            int len = tello->video_udp.read(buffer, sizeof(buffer));
            if (len > 0 && tello->videoDataCallback){
                // Call the video data callback function
                tello->videoDataCallback(buffer, len);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // Delay for 10ms
    }

    // Signal that the task has finished
    xSemaphoreGive(tello->videoTaskSemaphore);
    vTaskDelete(NULL); // Delete the task
}

// --- Enable Mission Pads ---
TelloStatus TelloESP32::enable_mission_pads() {
    return sendCommand("mon");
}

// --- Disable Mission Pads ---
TelloStatus TelloESP32::disable_mission_pads() {
    return sendCommand("moff");
}

// --- Set Mission Pad Detection Direction ---
TelloStatus TelloESP32::set_mission_pad_detection_direction(int direction) {
    if (direction < 0 || direction > 2) {
        return TelloStatus::InvalidParameter;
    }
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "mdirection %d", direction);
    return sendCommand(cmd);
}

// --- Set WiFi Credentials ---
TelloStatus TelloESP32::set_wifi_credentials(const char* ssid, const char* password) {
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "wifi %s %s", ssid, password);
    return sendCommand(cmd);
}

// --- Connect to WiFi (AP Mode) ---
TelloStatus TelloESP32::connect_to_wifi(const char* ssid, const char* password) {
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "ap %s %s", ssid, password);
    return sendCommand(cmd);
}

// --- Send Expansion Command ---
TelloStatus TelloESP32::send_expansion_command(const char* cmd) {
    char full_cmd[256];
    snprintf(full_cmd, sizeof(full_cmd), "EXT %s", cmd);
    return sendCommand(full_cmd);
}

bool TelloESP32::startVideoRecording(const char* filename) {
    // Open a file on the SD card using SD_MMC
    File videoFile = SD_MMC.open(filename, FILE_WRITE);
    if (!videoFile) {
        Serial.println("Failed to open file for writing");
        return false;
    }

    // Start receiving video packets and write to SD card
    // ...existing code...

    videoFile.close();
    return true;
}

} // namespace TelloControl