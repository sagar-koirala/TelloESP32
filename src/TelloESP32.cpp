/*
 * TelloESP32.cpp - Implementation of the TelloESP32 library
 *
 * Important Implementation Notes:
 * - Uses FreeRTOS tasks for concurrent operations
 * - Command responses handled in a separate task
 * - Video streaming runs on a dedicated task
 * - State updates occur every ~100ms
 * - Thread-safe response handling using a circular buffer and mutex
 * - Automatic reconnection attempts on connection loss (in receiveTask)
 * - Safe cleanup on disconnection (in disconnect)
 *
 * Resource Usage:
 * - Command task: 4KB stack
 * - Video task: 8KB stack
 * - UDP ports: 8889 (commands), 8890 (state), 11111 (video)
 * - Response buffer: 128 bytes per message x 10 messages (MAX_RESPONSES)
 *
 * Error Handling:
 * - All commands return TelloStatus
 * - Configurable error callback using setErrorCallback
 * - Connection timeout protection (CONNECTION_TIMEOUT)
 * - Parameter validation in movement and control commands
 *
 * Source code: https://github.com/sagar-koirala/TelloESP32.git
 * API reference: https://github.com/sagar-koirala/TelloESP32?tab=readme-ov-file#api-reference
 */

// TelloESP32.cpp
#include "TelloESP32.h"
#include <Arduino.h>
#include <WiFi.h>
// Other includes as needed by your project setup.
// <string.h> is typically not needed in Arduino projects as Arduino's String class
// and basic C-style string functions are available by default.

namespace TelloControl
{

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
        {"temph", &TelloESP32::set_temperature_high}};

    // --- Constructor Implementation ---
    /**
     * @brief Constructor for the TelloESP32 class.
     * Initializes member variables with default values, including network settings,
     * task handles, and synchronization primitives. Sets up a mutex for thread-safe
     * access to the response buffer and a semaphore for video task synchronization.
     */
    TelloESP32::TelloESP32() : telloAddr(192, 168, 10, 1),
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

    /**
     * @brief Destructor for the TelloESP32 class.
     * Ensures that video streaming is turned off, disconnects from the drone,
     * and cleans up the video task semaphore.
     */
    TelloESP32::~TelloESP32()
    {
        if (stream_on)
        {
            streamoff();
        }
        disconnect();
        vSemaphoreDelete(videoTaskSemaphore);
    }

    // --- Start Receive Task ---
    /**
     * @brief Starts the task responsible for receiving command responses and state data.
     * This task is pinned to core 0 and runs with a stack size of 4KB. It handles
     * incoming UDP packets and updates the drone's state accordingly.
     */
    void TelloESP32::startReceiveTask()
    {
        if (!responseTaskRunning && receiveTaskHandle == nullptr)
        {
            responseTaskRunning = true;
            // Create the response receive task (pinned to core 0)
            xTaskCreatePinnedToCore(
                receiveTask,        // Task function
                "TelloReceive",     // Task name
                4096,               // Stack size (4KB) - adjust as necessary
                this,               // Task parameter (this pointer)
                2,                  // Task priority
                &receiveTaskHandle, // Task handle
                0                   // Core ID (0 or 1) - pinned to core 0
            );
        }
    }

    // --- Stop Receive Task ---
    /**
     * @brief Stops the task responsible for receiving command responses and state data.
     * Signals the task to stop and waits for it to terminate before cleaning up.
     */
    void TelloESP32::stopReceiveTask()
    {
        if (responseTaskRunning && receiveTaskHandle != nullptr)
        {
            responseTaskRunning = false; // Signal the task to stop
            // Wait for the task to finish
            while (receiveTaskHandle != nullptr)
            {
                vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for the task to exit
            }
            // No need to call vTaskDelete here since the task deletes itself
        }
    }

    // --- Receive Task ---
    /**
     * @brief Task function for receiving command responses and state data from the Tello drone.
     * This task continuously listens for incoming UDP packets on both the command and state ports.
     * It processes received data, updates the drone's state, and checks for connection loss.
     * @param parameter A pointer to the TelloESP32 instance (passed as a void pointer).
     */
    void TelloESP32::receiveTask(void *parameter)
    {
        TelloESP32 *tello = static_cast<TelloESP32 *>(parameter);
        char cmd_buffer[MAX_RESPONSE_LENGTH]; // Buffer for command responses
        char state_buffer[256];                           // Buffer for state data
        unsigned long last_state_time = millis();         // Track the time of the last received state

        while (tello->responseTaskRunning)
        {
            // --- Check for command responses ---
            int cmd_packetSize = tello->udp.parsePacket();
            if (cmd_packetSize > 0 && cmd_packetSize <= MAX_RESPONSE_LENGTH)
            {
                int len = tello->udp.read(cmd_buffer, sizeof(cmd_buffer) - 1);
                if (len > 0)
                {
                    cmd_buffer[len] = '\0'; // Null-terminate the string

                    // --- Store response in the circular buffer (thread-safe) ---
                    portENTER_CRITICAL(&tello->responseMux);
                    int nextHead = (tello->responseHead + 1) % MAX_RESPONSES;
                    if (nextHead != tello->responseTail)
                    {
                        strncpy(tello->responses[tello->responseHead], cmd_buffer, sizeof(tello->responses[tello->responseHead]));
                        tello->responses[tello->responseHead][sizeof(tello->responses[tello->responseHead]) - 1] = '\0'; // Ensure null termination
                        tello->responseHead = nextHead;
                    }
                    portEXIT_CRITICAL(&tello->responseMux);
                }
            }
            
            // --- Check for state data ---
            int state_packetSize = tello->state_udp.parsePacket();
            if (state_packetSize > 0)
            {
                int len = tello->state_udp.read(state_buffer, sizeof(state_buffer) - 1);
                if (len > 0)
                {
                    state_buffer[len] = '\0';         // Null-terminate the string
                    tello->parse_state(state_buffer); // Parse the state data
                    last_state_time = millis();       // Update last received state time
                }
            }

            // --- Check for connection loss ---
            if (millis() - last_state_time > TelloESP32::CONNECTION_TIMEOUT)
            {
                tello->connected = false;
                // Reset all elements of the state struct to 0
                memset(&tello->state, 0, sizeof(tello->state));
                TELLO_LOGLN("Connection lost. State variables reset.");
            }

            vTaskDelay(10 / portTICK_PERIOD_MS); // Delay for 10ms
        }
        tello->receiveTaskHandle = nullptr; // Indicate that the task has exited
        vTaskDelete(NULL);                  // Self-delete the task
    }

    // --- Connection Management ---
    /**
     * @brief Establishes a connection to the Tello drone.
     *
     * Connects to the Tello's WiFi network, initializes UDP communication,
     * starts the receive task, and sends the initial command to enter SDK mode.
     *
     * @param ssid The SSID of the Tello drone's WiFi network.
     * @param password The password for the Tello's WiFi network (usually empty for Tello drones).
     * @param timeout_ms The maximum time (in milliseconds) to wait for the WiFi connection to be established.
     * @return A TelloStatus indicating the outcome of the connection attempt.
     *         Returns TelloStatus::OK if the connection is successful, or an appropriate error status otherwise.
     */
    TelloStatus TelloESP32::connect(const char *ssid, const char *password, unsigned long timeout_ms)
    {
        unsigned long startTime = millis();
        WiFi.begin(ssid, password);
        TELLO_LOG("Connecting to Tello");

        // --- Wait for WiFi connection ---
        while (WiFi.status() != WL_CONNECTED)
        {
            if (millis() - startTime > timeout_ms)
            {
                TELLO_LOGLN(" Connection timed out!");
                WiFi.disconnect(true);
                return TelloStatus::Timeout;
            }
            delay(350);
            TELLO_LOG(".");
        }

        // --- Initialize UDP ---
        if (!udp.begin(localPort))
        {
            TELLO_LOGLN("Failed to initialize command UDP.");
            WiFi.disconnect(true);
            return TelloStatus::UdpError;
        }
        if (!state_udp.begin(statePort))
        {
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
        if (status == TelloStatus::OK)
        {
            return TelloStatus::OK;
        }
        else
        {
            TELLO_LOGLN("Failed to activate SDK mode");
            disconnect(); // Clean up resources
            return status;
        }
    }

    // --- Wait Between Commands ---
    /**
     * @brief Ensures that a minimum delay is observed between consecutive commands sent to the Tello drone.
     * This function helps maintain stable communication with the drone by preventing command flooding.
     */
    void TelloESP32::waitBetweenCommands() const
    {
        unsigned long current_time = millis();
        unsigned long diff = current_time - last_received_command_timestamp;
        if (diff < TIME_BTW_COMMANDS)
        {
            delay(TIME_BTW_COMMANDS - diff);
        }
    }

    // --- error callback ---
    /**
     * @brief Sets a callback function to be called when an error occurs during command execution.
     * This allows for custom error handling by the user application.
     * @param callback The error callback function to be called.
     */
    void TelloESP32::setErrorCallback(ErrorCallback callback)
    {
        _errorCallback = callback;
    }

    // --- Internal Send Command (Action or Query) ---
    /**
     * @brief Sends a command to the Tello drone and handles the response.
     * This is an internal function used by both action commands (like `takeoff`, `land`) and query commands (like `battery?`).
     *
     * @param command The command string to send to the drone.
     * @param type The type of the command (Action or Query).
     * @return A TelloStatus indicating the outcome of the command.
     *         Returns `TelloStatus::OK` if the command was sent and acknowledged successfully.
     *         Returns `TelloStatus::NotConnected` if not connected to the drone.
     *         Returns `TelloStatus::NoResponse` if no response is received within the timeout period.
     *         For action commands, returns `TelloStatus::UnknownError` if the Tello responds with an error message, and in this case the `_errorCallback` is also called if set.
     */
    TelloStatus TelloESP32::_sendCommand(const char *command, CommandType type) const
    {
        if (!connected)
        {
            const_cast<TelloESP32 *>(this)->lastResponse = (type == CommandType::Query) ? "" : "Not connected";
            return TelloStatus::NotConnected;
        }

        waitBetweenCommands(); // Wait for the required time between commands

        // --- Send the command ---
        TELLO_LOG("Sending ");
        TELLO_LOG(type == CommandType::Query ? "query: " : "command: ");
        TELLO_LOG(command);
        const_cast<TelloESP32 *>(this)->udp.beginPacket(telloAddr, telloPort);
        const_cast<TelloESP32 *>(this)->udp.write((const uint8_t *)command, strlen(command));
        const_cast<TelloESP32 *>(this)->udp.endPacket();

        last_received_command_timestamp = millis(); // Update the timestamp of the last sent command

        // --- Wait for and handle the response (for action commands) ---
        if (type == CommandType::Action)
        {
            String response = waitForResponse(RESPONSE_TIMEOUT);
            if (response == "ok")
            {
                TELLO_LOG(" .... Response: ");
                TELLO_LOGLN(response);
                const_cast<TelloESP32 *>(this)->lastResponse = response;
                return TelloStatus::OK;
            }
            else if (response.startsWith("error"))
            {
                TELLO_LOG(" .... Error Response: ");
                TELLO_LOGLN(response);

                const_cast<TelloESP32 *>(this)->lastResponse = response;
                const_cast<TelloESP32 *>(this)->lastErrorMessage = response; // Store entire error message

                // Call error callback if registered
                if (const_cast<TelloESP32 *>(this)->_errorCallback)
                {
                    const_cast<TelloESP32 *>(this)->_errorCallback(command, response.c_str());
                }

                return TelloStatus::UnknownError; // Return unknown error status
            }
            else
            {
                TELLO_LOGLN("  !!! No response received.");
                const_cast<TelloESP32 *>(this)->lastResponse = "No response";
                return TelloStatus::NoResponse;
            }
        }
        else
        {
            // For query commands, the response is handled by sendQueryCommand()
            String response = waitForResponse(RESPONSE_TIMEOUT);
            const_cast<TelloESP32 *>(this)->lastResponse = response;
            return TelloStatus::OK;
        }
    }

    // --- Send a Command ---
    /**
     * @brief Sends an action command to the Tello drone.
     * @param command The command string to send (e.g., "takeoff", "land", "forward 50").
     * @return A TelloStatus indicating the result of the command.
     */
    TelloStatus TelloESP32::sendCommand(const char *command)
    {
        return _sendCommand(command, CommandType::Action);
    }

    // --- Send a Query Command and Return the Response ---
    /**
     * @brief Sends a query command to the Tello drone and returns the response.
     * @param command The query command string to send (e.g., "battery?", "speed?").
     * @return The response from the Tello drone as a String. Returns an empty string on error.
     */
    String TelloESP32::sendQueryCommand(const char *command) const
    {
        if (_sendCommand(command, CommandType::Query) == TelloStatus::OK)
        {
            return lastResponse;
        }
        return ""; // Return an empty string on error
    }

    // --- Takeoff Command ---
    /**
     * @brief Sends the "takeoff" command to the Tello drone.
     * @return A TelloStatus indicating the result of the takeoff command.
     */
    TelloStatus TelloESP32::takeoff()
    {
        if (!connected)
            return TelloStatus::NotConnected;
        if (is_flying)
        {
            TELLO_LOGLN("Already flying");
            return TelloStatus::OK;
        }

        TelloStatus status = sendCommand("takeoff");
        if (status == TelloStatus::OK)
            is_flying = true;
        return status;
    }

    // --- Land Command ---
    /**
     * @brief Sends the "land" command to the Tello drone.
     * @return A TelloStatus indicating the result of the land command.
     */
    TelloStatus TelloESP32::land()
    {
        if (!connected)
            return TelloStatus::NotConnected;
        if (!is_flying)
        {
            TELLO_LOGLN("Already landed");
            return TelloStatus::OK;
        }

        TelloStatus status = sendCommand("land");
        if (status == TelloStatus::OK)
            is_flying = false;
        return status;
    }

    // --- Movement Commands with Range Checks ---
    /**
     * @brief Moves the Tello drone upward.
     *
     * @param cm The distance to move up in centimeters. Must be between 20 and 500.
     * @return A TelloStatus indicating the result of the command.
     *         Returns TelloStatus::InvalidParameter if the distance is out of range.
     */
    TelloStatus TelloESP32::move_up(int cm)
    {
        if (cm < 20 || cm > 500)
            return TelloStatus::InvalidParameter;
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "up %d", cm);
        return sendCommand(cmd);
    }

    /**
     * @brief Moves the Tello drone downward.
     *
     * @param cm The distance to move down in centimeters. Must be between 20 and 500.
     * @return A TelloStatus indicating the result of the command.
     *         Returns TelloStatus::InvalidParameter if the distance is out of range.
     */
    TelloStatus TelloESP32::move_down(int cm)
    {
        if (cm < 20 || cm > 500)
            return TelloStatus::InvalidParameter;
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "down %d", cm);
        return sendCommand(cmd);
    }

    /**
     * @brief Moves the Tello drone to the left.
     *
     * @param cm The distance to move left in centimeters. Must be between 20 and 500.
     * @return A TelloStatus indicating the result of the command.
     *         Returns TelloStatus::InvalidParameter if the distance is out of range.
     */
    TelloStatus TelloESP32::move_left(int cm)
    {
        if (cm < 20 || cm > 500)
            return TelloStatus::InvalidParameter;
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "left %d", cm);
        return sendCommand(cmd);
    }

    /**
     * @brief Moves the Tello drone to the right.
     *
     * @param cm The distance to move right in centimeters. Must be between 20 and 500.
     * @return A TelloStatus indicating the result of the command.
     *         Returns TelloStatus::InvalidParameter if the distance is out of range.
     */
    TelloStatus TelloESP32::move_right(int cm)
    {
        if (cm < 20 || cm > 500)
            return TelloStatus::InvalidParameter;
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "right %d", cm);
        return sendCommand(cmd);
    }

    /**
     * @brief Moves the Tello drone forward.
     *
     * @param cm The distance to move forward in centimeters. Must be between 20 and 500.
     * @return A TelloStatus indicating the result of the command.
     *         Returns TelloStatus::InvalidParameter if the distance is out of range.
     */
    TelloStatus TelloESP32::move_forward(int cm)
    {
        if (cm < 20 || cm > 500)
            return TelloStatus::InvalidParameter;
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "forward %d", cm);
        return sendCommand(cmd);
    }

    /**
     * @brief Moves the Tello drone backward.
     *
     * @param cm The distance to move back in centimeters. Must be between 20 and 500.
     * @return A TelloStatus indicating the result of the command.
     *         Returns TelloStatus::InvalidParameter if the distance is out of range.
     */
    TelloStatus TelloESP32::move_back(int cm)
    {
        if (cm < 20 || cm > 500)
            return TelloStatus::InvalidParameter;
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "back %d", cm);
        return sendCommand(cmd);
    }

    /**
     * @brief Rotates the Tello drone clockwise.
     *
     * @param degrees The angle to rotate in degrees. Must be between 1 and 360.
     * @return A TelloStatus indicating the result of the command.
     *         Returns TelloStatus::InvalidParameter if the angle is out of range.
     */
    TelloStatus TelloESP32::rotate_clockwise(int degrees)
    {
        if (degrees < 1 || degrees > 360)
            return TelloStatus::InvalidParameter;
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "cw %d", degrees);
        return sendCommand(cmd);
    }

    /**
     * @brief Rotates the Tello drone counter-clockwise.
     *
     * @param degrees The angle to rotate in degrees. Must be between 1 and 360.
     * @return A TelloStatus indicating the result of the command.
     *         Returns TelloStatus::InvalidParameter if the angle is out of range.
     */
    TelloStatus TelloESP32::rotate_counter_clockwise(int degrees)
    {
        if (degrees < 1 || degrees > 360)
            return TelloStatus::InvalidParameter;
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "ccw %d", degrees);
        return sendCommand(cmd);
    }

    // --- Flip Commands ---
    /**
     * @brief Makes the Tello drone perform a flip to the left.
     * @return A TelloStatus indicating the result of the command.
     */
    TelloStatus TelloESP32::flip_left()
    {
        return sendCommand("flip l");
    }

    /**
     * @brief Makes the Tello drone perform a flip to the right.
     * @return A TelloStatus indicating the result of the command.
     */
    TelloStatus TelloESP32::flip_right()
    {
        return sendCommand("flip r");
    }

    /**
     * @brief Makes the Tello drone perform a flip forward.
     * @return A TelloStatus indicating the result of the command.
     */
    TelloStatus TelloESP32::flip_forward()
    {
        return sendCommand("flip f");
    }

    /**
     * @brief Makes the Tello drone perform a flip backward.
     * @return A TelloStatus indicating the result of the command.
     */
    TelloStatus TelloESP32::flip_back()
    {
        return sendCommand("flip b");
    }

    // --- Disconnect from the Tello ---
    /**
     * @brief Disconnects from the Tello drone.
     *
     * Lands the drone if it is flying, stops the receive and video tasks,
     * closes the UDP sockets, disconnects from the WiFi network, and resets
     * the drone's state variables.
     */
    void TelloESP32::disconnect()
    {
        if (connected)
        {
            if (is_flying)
            {
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
    /**
     * @brief Waits for a response from the Tello drone for a specified amount of time.
     *
     * @param timeout_ms The maximum time to wait for a response, in milliseconds.
     * @return The response string received from the drone.
     *         Returns an empty string if no response is received within the timeout period.
     */
    String TelloESP32::waitForResponse(unsigned long timeout_ms) const
    {
        unsigned long startTime = millis();

        while (millis() - startTime < timeout_ms)
        {
            portENTER_CRITICAL(&responseMux);
            if (responseHead != responseTail)
            {
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
    /**
     * @brief Returns the last response string received from the Tello drone.
     * @return The last response string.
     */
    String TelloESP32::getLastResponse() const
    {
        return lastResponse;
    }

    /**
     * @brief Returns the last error message received or generated.
     * @return The last error message.
     */
    String TelloESP32::getLastErrorMessage() const
    {
        return lastErrorMessage;
    }

    /**
     * @brief Clears the last error message and the last response.
     */
    void TelloESP32::clearLastError()
    {
        lastErrorMessage = "";
        lastResponse = "";
    }

    // --- Emergency Stop ---
    /**
     * @brief Sends the "emergency" command to the Tello drone, which immediately stops all motors.
     * Use this command with extreme caution, as the drone will fall if it is in the air.
     */
    void TelloESP32::emergency()
    {
        sendCommand("emergency");
        is_flying = false;
    }

    // --- Turn Motor On ---
    /**
     * @brief Sends the "motoron" command to the Tello drone. This is typically used for cooling the motors.
     * @return A TelloStatus indicating the result of the command.
     */
    TelloStatus TelloESP32::turn_motor_on()
    {
        return sendCommand("motoron");
    }

    // --- Turn Motor Off ---
    /**
     * @brief Sends the "motoroff" command to the Tello drone, turning off the motors.
     * @return A TelloStatus indicating the result of the command.
     */
    TelloStatus TelloESP32::turn_motor_off()
    {
        return sendCommand("motoroff");
    }

    // --- Go to XYZ with Speed ---
    /**
     * @brief Sends the "go" command to the Tello drone, making it move to a specific coordinate with a given speed.
     *
     * @param x The x-coordinate of the target location. Must be between -500 and 500.
     * @param y The y-coordinate of the target location. Must be between -500 and 500.
     * @param z The z-coordinate of the target location. Must be between -500 and 500.
     * @param speed The speed at which to move to the target location. Must be between 10 and 100.
     * @return A TelloStatus indicating the result of the command.
     *         Returns TelloStatus::InvalidParameter if any of the parameters are out of range.
     */
    TelloStatus TelloESP32::go_xyz_speed(int x, int y, int z, int speed)
    {
        if (x < -500 || x > 500 || y < -500 || y > 500 || z < -500 || z > 500 ||
            speed < 10 || speed > 100)
        {
            return TelloStatus::InvalidParameter;
        }
        char cmd[64];
        snprintf(cmd, sizeof(cmd), "go %d %d %d %d", x, y, z, speed);
        return sendCommand(cmd);
    }

    // --- Curve at XYZ with Speed ---
    /**
     * @brief Sends the "curve" command to the Tello drone, making it move along a curved path defined by two points with a given speed.
     *
     * @param x1 The x-coordinate of the first point. Must be between -500 and 500.
     * @param y1 The y-coordinate of the first point. Must be between -500 and 500.
     * @param z1 The z-coordinate of the first point. Must be between -500 and 500.
     * @param x2 The x-coordinate of the second point. Must be between -500 and 500.
     * @param y2 The y-coordinate of the second point. Must be between -500 and 500.
     * @param z2 The z-coordinate of the second point. Must be between -500 and 500.
     * @param speed The speed at which to move along the curve. Must be between 10 and 60.
     * @return A TelloStatus indicating the result of the command.
     *         Returns TelloStatus::InvalidParameter if any of the parameters are out of range.
     */
    TelloStatus TelloESP32::curve_xyz_speed(int x1, int y1, int z1,
                                            int x2, int y2, int z2, int speed)
    {
        if (x1 < -500 || x1 > 500 || y1 < -500 || y1 > 500 || z1 < -500 || z1 > 500 ||
            x2 < -500 || x2 > 500 || y2 < -500 || y2 > 500 || z2 < -500 || z2 > 500 ||
            speed < 10 || speed > 60)
        {
            return TelloStatus::InvalidParameter;
        }
        char cmd[128];
        snprintf(cmd, sizeof(cmd), "curve %d %d %d %d %d %d %d",
                 x1, y1, z1, x2, y2, z2, speed);
        return sendCommand(cmd);
    }

    // --- Set Speed ---
    /**
     * @brief Sends the "speed" command to the Tello drone to set its default speed.
     *
     * @param speed The desired speed. Must be between 10 and 100.
     * @return A TelloStatus indicating the result of the command.
     *         Returns TelloStatus::InvalidParameter if the speed is out of range.
     */
    TelloStatus TelloESP32::set_speed(int speed)
    {
        if (speed < 10 || speed > 100)
        {
            return TelloStatus::InvalidParameter;
        }
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "speed %d", speed);
        return sendCommand(cmd);
    }

    // --- Send RC Control ---
    /**
     * @brief Sends RC control commands to the Tello drone.
     *
     * This function allows for direct control of the drone's roll, pitch, yaw, and throttle.
     * The values should be in the range of -100 to 100.
     * This command must be sent repeatedly (e.g., every 50ms) to maintain smooth control.
     *
     * @param left_right_velocity The left/right velocity (roll).
     * @param forward_backward_velocity The forward/backward velocity (pitch).
     * @param up_down_velocity The up/down velocity (throttle).
     * @param yaw_velocity The yaw velocity (rotation).
     */
    void TelloESP32::send_rc_control(int left_right_velocity,
                                     int forward_backward_velocity,
                                     int up_down_velocity,
                                     int yaw_velocity)
    {
        if (!connected)
            return;

        unsigned long current_time = millis();
        if (current_time - last_rc_control_timestamp > TIME_BTW_RC_CONTROL_COMMANDS)
        {
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
    /**
     * @brief Queries the current speed setting of the Tello drone.
     * @return The current speed setting (cm/s). Returns 0 on error.
     */
    int TelloESP32::query_speed() const
    {
        return stringToInt(sendQueryCommand("speed?").c_str());
    }

    /**
     * @brief Queries the current battery percentage of the Tello drone.
     * @return The current battery percentage. Returns 0 on error.
     */
    int TelloESP32::query_battery() const
    {
        return stringToInt(sendQueryCommand("battery?").c_str());
    }

    /**
     * @brief Queries the current flight time of the Tello drone.
     * @return The current flight time in seconds. Returns 0 on error.
     */
    int TelloESP32::query_time() const
    {
        return stringToInt(sendQueryCommand("time?").c_str());
    }

    /**
     * @brief Queries the current height of the Tello drone.
     * @return The current height in decimeters (dm). Returns 0 on error.
     */
    int TelloESP32::query_height() const
    {
        return stringToInt(sendQueryCommand("height?").c_str());
    }

    /**
     * @brief Queries the current temperature of the Tello drone.
     * If a temperature range is provided, returns the average of the range.
     * @return The current temperature (average if a range) in Celsius. Returns 0 on error.
     */
    int TelloESP32::query_temp() const
    {
        String response = sendQueryCommand("temp?");
        if (response.indexOf('~') != -1)
        {
            int minTemp = stringToInt(response.substring(0, response.indexOf('~')).c_str());
            int maxTemp = stringToInt(response.substring(response.indexOf('~') + 1, response.length()).c_str());
            return (minTemp + maxTemp) / 2;
        }
        else
        {
            return stringToInt(response.c_str());
        }
    }

    /**
     * @brief Queries the attitude (pitch, roll, yaw) of the Tello drone.
     * @return A string containing the attitude values in the format "pitch:0;roll:0;yaw:0". Returns an empty string on error.
     */
    String TelloESP32::query_attitude() const
    {
        return sendQueryCommand("attitude?");
    }

    /**
     * @brief Queries the barometer reading of the Tello drone.
     * @return The barometer reading in meters. Returns 0.0f on error.
     */
    float TelloESP32::query_baro() const
    {
        return stringToFloat(sendQueryCommand("baro?").c_str());
    }

    /**
     * @brief Queries the time-of-flight distance of the Tello drone.
     * @return The time-of-flight distance in centimeters. Returns 0.0f on error.
     */
    float TelloESP32::query_tof() const
    {
        return stringToFloat(sendQueryCommand("tof?").c_str());
    }

    /**
     * @brief Queries the WiFi signal-to-noise ratio (SNR) of the Tello drone.
     * @return A string containing the WiFi SNR (e.g., "snr:90"). Returns an empty string on error.
     */
    String TelloESP32::query_wifi() const
    {
        return sendQueryCommand("wifi?");
    }

    /**
     * @brief Queries the SDK version of the Tello drone.
     * @return A string containing the SDK version (e.g., "SDK:2.0"). Returns an empty string on error.
     */
    String TelloESP32::query_sdk() const
    {
        return sendQueryCommand("sdk?");
    }

    /**
     * @brief Queries the serial number of the Tello drone.
     * @return A string containing the serial number. Returns an empty string on error.
     */
    String TelloESP32::query_sn() const
    {
        return sendQueryCommand("sn?");
    }

    // --- Direct State Getters (const methods) ---
    /**
     * @brief Gets the current height of the drone (cm). Faster than query_height.
     * @return Current height in cm.
     */
    int TelloESP32::get_height() const { return state.height; }

    /**
     * @brief Gets the current battery percentage of the drone. Faster than query_battery.
     * @return Current battery percentage.
     */
    int TelloESP32::get_battery() const { return state.battery; }

    /**
     * @brief Gets the current flight time of the drone. Faster than query_time.
     * @return Current flight time in seconds.
     */
    int TelloESP32::get_flight_time() const { return state.flight_time; }

    /**
     * @brief Gets the current temperature of the drone (average of high and low temperature). Faster than query_temp.
     * @return Current temperature in Celsius.
     */
    float TelloESP32::get_temperature() const { return (state.temperature_low + state.temperature_high) / 2.0f; }

    /**
     * @brief Gets the current barometer reading of the drone. Faster than query_baro.
     * @return Current barometer reading in meters.
     */
    float TelloESP32::get_barometer() const { return state.barometer; }

    /**
     * @brief Gets the current TOF distance of the drone. Faster than query_tof.
     * @return Current TOF distance in cm.
     */
    float TelloESP32::get_tof() const { return state.tof; }

    /**
     * @brief Gets the current pitch of the drone.
     * @return Current pitch in degrees.
     */
    int TelloESP32::get_pitch() const { return state.pitch; }

    /**
     * @brief Gets the current roll of the drone.
     * @return Current roll in degrees.
     */
    int TelloESP32::get_roll() const { return state.roll; }

    /**
     * @brief Gets the current yaw of the drone.
     * @return Current yaw in degrees.
     */
    int TelloESP32::get_yaw() const { return state.yaw; }

    /**
     * @brief Gets the current x-axis speed of the drone.
     * @return Current x-axis speed in cm/s.
     */
    int TelloESP32::get_speed_x() const { return state.speed_x; }

    /**
     * @brief Gets the current y-axis speed of the drone.
     * @return Current y-axis speed in cm/s.
     */
    int TelloESP32::get_speed_y() const { return state.speed_y; }

    /**
     * @brief Gets the current z-axis speed of the drone.
     * @return Current z-axis speed in cm/s.
     */
    int TelloESP32::get_speed_z() const { return state.speed_z; }

    /**
     * @brief Gets the current x-axis acceleration of the drone.
     * @return Current x-axis acceleration in cm/s^2.
     */
    float TelloESP32::get_acceleration_x() const { return state.accel_x; }

    /**
     * @brief Gets the current y-axis acceleration of the drone.
     * @return Current y-axis acceleration in cm/s^2.
     */
    float TelloESP32::get_acceleration_y() const { return state.accel_y; }

    /**
     * @brief Gets the current z-axis acceleration of the drone.
     * @return Current z-axis acceleration in cm/s^2.
     */
    float TelloESP32::get_acceleration_z() const { return state.accel_z; }

    // --- State Parsing Function (using strtok for efficiency) ---
    /**
     * @brief Parses the state data string received from the Tello drone and updates the internal state variables.
     *
     * The state data string is expected to be in the format "key1:value1;key2:value2;...".
     * This method uses strtok for efficient tokenizing.
     * @param state_data The state data string to parse.
     */
    void TelloESP32::parse_state(const char *state_data) const
    {
        // Print raw state message if enabled
        if (print_Raw_State)
        {
            TELLO_LOG("Raw State: ");
            TELLO_LOGLN(state_data);
        }

        char *data_copy = strdup(state_data); // Create a copy for strtok to modify
        char *token = strtok(data_copy, ";");

        while (token != NULL)
        {
            char *separator = strchr(token, VALUE_DELIMITER);
            if (separator != NULL)
            {
                *separator = '\0'; // Null-terminate the key
                const char *key = token;
                const char *value = separator + 1;

                // Find the corresponding state field and call its setter
                for (const auto &mapping : STATE_MAPPINGS)
                {
                    if (strcmp(key, mapping.name) == 0)
                    {
                        (const_cast<TelloESP32 *>(this)->*mapping.setter)(value);
                        break;
                    }
                }
            }
            token = strtok(NULL, ";");
        }
        free(data_copy); // Free the allocated memory
    }

    // --- State Setters ---
    /**
     * @brief Sets the height value based on the received string.
     * @param value The value string to parse.
     */
    void TelloESP32::set_height(const char *value) const { const_cast<TelloESP32 *>(this)->state.height = stringToInt(value); }

    /**
     * @brief Sets the battery percentage based on the received string.
     * @param value The value string to parse.
     */
    void TelloESP32::set_battery(const char *value) const { const_cast<TelloESP32 *>(this)->state.battery = stringToInt(value); }

    /**
     * @brief Sets the flight time based on the received string.
     * @param value The value string to parse.
     */
    void TelloESP32::set_flight_time(const char *value) const { const_cast<TelloESP32 *>(this)->state.flight_time = stringToInt(value); }

    /**
     * @brief Sets the barometer reading based on the received string.
     * @param value The value string to parse.
     */
    void TelloESP32::set_barometer(const char *value) const { const_cast<TelloESP32 *>(this)->state.barometer = stringToFloat(value); }

    /**
     * @brief Sets the time-of-flight distance based on the received string.
     * @param value The value string to parse.
     */
    void TelloESP32::set_tof(const char *value) const { const_cast<TelloESP32 *>(this)->state.tof = stringToFloat(value); }

    /**
     * @brief Sets the pitch based on the received string.
     * @param value The value string to parse.
     */
    void TelloESP32::set_pitch(const char *value) const { const_cast<TelloESP32 *>(this)->state.pitch = stringToInt(value); }

    /**
     * @brief Sets the roll based on the received string.
     * @param value The value string to parse.
     */
    void TelloESP32::set_roll(const char *value) const { const_cast<TelloESP32 *>(this)->state.roll = stringToInt(value); }

    /**
     * @brief Sets the yaw based on the received string.
     * @param value The value string to parse.
     */
    void TelloESP32::set_yaw(const char *value) const { const_cast<TelloESP32 *>(this)->state.yaw = stringToInt(value); }

    /**
     * @brief Sets the x-axis speed based on the received string.
     * @param value The value string to parse.
     */
    void TelloESP32::set_speed_x(const char *value) const { const_cast<TelloESP32 *>(this)->state.speed_x = stringToInt(value); }

    /**
     * @brief Sets the y-axis speed based on the received string.
     * @param value The value string to parse.
     */
    void TelloESP32::set_speed_y(const char *value) const { const_cast<TelloESP32 *>(this)->state.speed_y = stringToInt(value); }

    /**
     * @brief Sets the z-axis speed based on the received string.
     * @param value The value string to parse.
     */
    void TelloESP32::set_speed_z(const char *value) const { const_cast<TelloESP32 *>(this)->state.speed_z = stringToInt(value); }

    /**
     * @brief Sets the x-axis acceleration based on the received string.
     * @param value The value string to parse.
     */
    void TelloESP32::set_acceleration_x(const char *value) const { const_cast<TelloESP32 *>(this)->state.accel_x = stringToFloat(value); }

    /**
     * @brief Sets the y-axis acceleration based on the received string.
     * @param value The value string to parse.
     */
    void TelloESP32::set_acceleration_y(const char *value) const { const_cast<TelloESP32 *>(this)->state.accel_y = stringToFloat(value); }

    /**
     * @brief Sets the z-axis acceleration based on the received string.
     * @param value The value string to parse.
     */
    void TelloESP32::set_acceleration_z(const char *value) const { const_cast<TelloESP32 *>(this)->state.accel_z = stringToFloat(value); }

    /**
     * @brief Sets the lower bound temperature based on the received string.
     * @param value The value string to parse.
     */
    void TelloESP32::set_temperature_low(const char *value) const { const_cast<TelloESP32 *>(this)->state.temperature_low = stringToInt(value); }

    /**
     * @brief Sets the upper bound temperature based on the received string.
     * @param value The value string to parse.
     */
    void TelloESP32::set_temperature_high(const char *value) const { const_cast<TelloESP32 *>(this)->state.temperature_high = stringToInt(value); }

    // --- String Conversion Helpers ---
    /**
     * @brief Converts a C-style string to an integer.
     * @param str The string to convert.
     * @return The converted integer value. Returns 0 if the string is NULL or empty.
     */
    int TelloESP32::stringToInt(const char *str)
    {
        if (str == NULL || *str == '\0')
        {
            return 0;
        }
        return atoi(str);
    }

    /**
     * @brief Converts a C-style string to a float.
     * @param str The string to convert.
     * @return The converted float value. Returns 0.0f if the string is NULL or empty.
     */
    float TelloESP32::stringToFloat(const char *str)
    {
        if (str == NULL || *str == '\0')
        {
            return 0.0f;
        }
        return atof(str);
    }

    // --- Clamp Value between -100 and 100 ---
    /**
     * @brief Clamps an integer value between -100 and 100.
     * @param value The value to clamp.
     * @return The clamped value.
     */
    int TelloESP32::clamp100(int value) const
    {
        return (value > 100) ? 100 : ((value < -100) ? -100 : value);
    }

    // --- Set Video Bitrate ---
    /**
     * @brief Sets the video bitrate for the Tello drone.
     * @param bitrate The desired bitrate.
     * @return A TelloStatus indicating the result of the command.
     */
    TelloStatus TelloESP32::set_video_bitrate(VideoBitrate bitrate)
    {
        if (!connected)
            return TelloStatus::NotConnected;

        char cmd[32];
        snprintf(cmd, sizeof(cmd), "setbitrate %d", static_cast<int>(bitrate));
        return sendCommand(cmd);
    }

    // --- Set Video Resolution ---
    /**
     * @brief Sets the video resolution for the Tello drone.
     * @param resolution The desired video resolution.
     * @return A TelloStatus indicating the result of the command.
     */
    TelloStatus TelloESP32::set_video_resolution(VideoResolution resolution)
    {
        if (!connected)
            return TelloStatus::NotConnected;

        const char *resolution_str;
        switch (resolution)
        {
        case VideoResolution::RESOLUTION_720P:
            resolution_str = "high";
            break;
        case VideoResolution::RESOLUTION_480P:
            resolution_str = "low";
            break;
        default:
            return TelloStatus::InvalidParameter;
        }

        char cmd[32];
        snprintf(cmd, sizeof(cmd), "setresolution %s", resolution_str);
        return sendCommand(cmd);
    }

    // --- Set Video FPS ---
    /**
     * @brief Sets the video frames per second (FPS) for the Tello drone.
     * @param fps The desired video FPS.
     * @return A TelloStatus indicating the result of the command.
     */
    TelloStatus TelloESP32::set_video_fps(VideoFPS fps)
    {
        if (!connected)
            return TelloStatus::NotConnected;

        const char *fps_str;
        switch (fps)
        {
        case VideoFPS::FPS_5:
            fps_str = "low";
            break;
        case VideoFPS::FPS_15:
            fps_str = "middle";
            break;
        case VideoFPS::FPS_30:
            fps_str = "high";
            break;
        default:
            return TelloStatus::InvalidParameter;
        }

        char cmd[32];
        snprintf(cmd, sizeof(cmd), "setfps %s", fps_str);
        return sendCommand(cmd);
    }

    // --- Set Video Direction ---
    /**
     * @brief Sets the video direction for the Tello drone's camera.
     * @param direction The desired camera direction (forward or downward).
     * @return A TelloStatus indicating the result of the command.
     */
    TelloStatus TelloESP32::set_video_direction(CameraDirection direction)
    {
        if (!connected)
            return TelloStatus::NotConnected;

        char cmd[32];
        snprintf(cmd, sizeof(cmd), "downvision %d", static_cast<int>(direction));
        return sendCommand(cmd);
    }

    // --- Stream On ---
    /**
     * @brief Turns on the video stream of the Tello drone.
     * @return A TelloStatus indicating the result of the command.
     */
    TelloStatus TelloESP32::streamon()
    {
        if (!connected)
            return TelloStatus::NotConnected;

        TelloStatus status = sendCommand("streamon");
        if (status == TelloStatus::OK)
        {
            stream_on = true;
            startVideoReceiveTask(); // Start video receive task
        }
        return status;
    }

    // --- Stream Off ---
    /**
     * @brief Turns off the video stream of the Tello drone.
     * @return A TelloStatus indicating the result of the command.
     */
    TelloStatus TelloESP32::streamoff()
    {
        if (!connected)
            return TelloStatus::NotConnected;

        TelloStatus status = sendCommand("streamoff");
        if (status == TelloStatus::OK)
        {
            stream_on = false;
            stopVideoReceiveTask(); // Stop video receive task
        }
        return status;
    }

    // --- Set Video Data Callback ---
    /**
     * @brief Sets the video data callback function.
     * @param callback The function to be called when video data is received.
     */
    void TelloESP32::setVideoDataCallback(VideoDataCallback callback)
    {
        videoDataCallback = callback;
    }

    // --- Start Video Receive Task ---
    /**
     * @brief Starts the task for receiving video data from the Tello drone.
     * This function initializes the video UDP port and creates a new task that
     * handles the incoming video stream and calls the user-provided callback.
     */
    void TelloESP32::startVideoReceiveTask()
    {
        if (!videoTaskRunning && videoReceiveTaskHandle == nullptr)
        {
            videoTaskRunning = true;
            xSemaphoreGive(videoTaskSemaphore); // Reset semaphore state

            // Begin listening on the video stream port
            if (!video_udp.begin(videoPort))
            {
                TELLO_LOGLN("Failed to initialize video UDP.");
                return;
            }

            xTaskCreate(
                videoReceiveTask,       // Task function
                "TelloVideoReceive",    // Task name
                8192,                   // Stack size (8KB) - adjust as necessary
                this,                   // Task parameter (this pointer)
                2,                      // Task priority
                &videoReceiveTaskHandle // Task handle
            );
        }
    }

    // --- Stop Video Receive Task ---
    /**
     * @brief Stops the video receive task.
     * This function signals the video task to stop and waits for it to terminate, ensuring
     * proper cleanup and preventing any resource leaks. Also handles cases where
     * task does not terminate gracefully.
     */
    void TelloESP32::stopVideoReceiveTask()
    {
        if (videoTaskRunning && videoReceiveTaskHandle != nullptr)
        {
            videoTaskRunning = false; // Signal the task to stop

            // Wait for the task to finish (with timeout)
            if (xSemaphoreTake(videoTaskSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE)
            {
                // Stop UDP after the task has finished
                video_udp.stop();

                // Now safe to delete the task
                vTaskDelete(videoReceiveTaskHandle);
                videoReceiveTaskHandle = nullptr;
            }
            else
            {
                // Emergency cleanup if the task doesn't respond
                TELLO_LOGLN("Warning: Video task did not respond to stop signal.");
                video_udp.stop();
                vTaskDelete(videoReceiveTaskHandle);
                videoReceiveTaskHandle = nullptr;
            }
        }
    }

    // --- Video Receive Task ---
    /**
     * @brief Task function for receiving video data from the Tello drone.
     * This task continuously reads incoming video data from the UDP socket and calls
     * the user-provided video data callback function with the received data.
     * @param parameter A pointer to the TelloESP32 instance (passed as a void pointer).
     */
    void TelloESP32::videoReceiveTask(void *parameter)
    {
        TelloESP32 *tello = static_cast<TelloESP32 *>(parameter);
        uint8_t buffer[2048]; // Buffer for video data (adjust size as needed)

        while (tello->videoTaskRunning)
        {
            int packetSize = tello->video_udp.parsePacket();
            if (packetSize > 0)
            {
                int len = tello->video_udp.read(buffer, sizeof(buffer));
                if (len > 0 && tello->videoDataCallback)
                {
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
    /**
     * @brief Enables the mission pad detection feature on the Tello drone.
     * @return A TelloStatus indicating the result of the command.
     */
    TelloStatus TelloESP32::enable_mission_pads()
    {
        return sendCommand("mon");
    }

    // --- Disable Mission Pads ---
    /**
     * @brief Disables the mission pad detection feature on the Tello drone.
     * @return A TelloStatus indicating the result of the command.
     */
    TelloStatus TelloESP32::disable_mission_pads()
    {
        return sendCommand("moff");
    }

    // --- Set Mission Pad Detection Direction ---
    /**
     * @brief Sets the direction for mission pad detection on the Tello drone.
     * @param direction 0 for both downward and forward detection, 1 for downward only, 2 for forward only.
     * @return A TelloStatus indicating the result of the command. Returns TelloStatus::InvalidParameter if the direction value is invalid.
     */
    TelloStatus TelloESP32::set_mission_pad_detection_direction(int direction)
    {
        if (direction < 0 || direction > 2)
        {
            return TelloStatus::InvalidParameter;
        }
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "mdirection %d", direction);
        return sendCommand(cmd);
    }

    // --- Set WiFi Credentials ---
    /**
     * @brief Sets the WiFi credentials on the Tello drone (for station mode).
     * @param ssid The SSID of the network to connect to.
     * @param password The password of the network.
     * @return A TelloStatus indicating the result of the command.
     */
    TelloStatus TelloESP32::set_wifi_credentials(const char *ssid, const char *password)
    {
        char cmd[128];
        snprintf(cmd, sizeof(cmd), "wifi %s %s", ssid, password);
        return sendCommand(cmd);
    }

    // --- Connect to WiFi (AP Mode) ---
    /**
     * @brief Connects the Tello drone to a WiFi network (station mode).
     * @param ssid The SSID of the network to connect to.
     * @param password The password of the network.
     * @return A TelloStatus indicating the result of the command.
     */
    TelloStatus TelloESP32::connect_to_wifi(const char *ssid, const char *password)
    {
        char cmd[128];
        snprintf(cmd, sizeof(cmd), "ap %s %s", ssid, password);
        return sendCommand(cmd);
    }

    // --- Send Expansion Command ---
    /**
     * @brief Sends a command to the Tello expansion board.
     * @param cmd The command string to send.
     * @return A TelloStatus indicating the result of the command.
     */
    TelloStatus TelloESP32::send_expansion_command(const char *cmd)
    {
        char full_cmd[256];
        snprintf(full_cmd, sizeof(full_cmd), "EXT %s", cmd);
        return sendCommand(full_cmd);
    }

    /**
     * @brief Start the video stream.
     * This function sends the `streamon` command and starts the video receiving task.
     */
    void TelloESP32::startVideoStream()
    {
        streamon();
    }
    
} // namespace TelloControl