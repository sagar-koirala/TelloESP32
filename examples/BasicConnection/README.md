# Basic Connection Example

This example demonstrates how to establish a basic connection with a Tello drone using the TelloESP32 library. It covers connection, status checking, and proper disconnection procedures.

Source code: https://github.com/sagar-koirala/TelloESP32.git

## Usage Notes
1. Set your Serial Monitor baud rate to 115200
2. Replace `TELLO_SSID` with your Tello's SSID if different
3. Always ensures safe disconnection

## Expected Serial Output
When running correctly, you should see output similar to this (timestamps will vary):
```
09:37:15.372 -> TelloESP32 Basic Connection Example
09:37:15.912 -> Connecting to Tello drone: TELLO-56CC13
09:37:25.491 -> Successfully connected to Tello!
09:37:30.024 -> Disconnecting from Tello...
09:37:34.068 -> E (66705) wifi:NAN WiFi stop
09:37:35.241 -> Disconnected.
```

## Available Functions
### Connection Management

*   **`TelloStatus connect(const char* ssid, const char* password, unsigned long timeout_ms = 10000)`:**  
    Connects to the Tello drone. Returns connection status.
    - `ssid`: The Tello's SSID (usually starting with "TELLO-")
    - `password`: Usually empty for Tello
    - `timeout_ms`: Connection timeout in milliseconds (optional)

*   **`void disconnect()`:**  
    Safely disconnects from the Tello drone.

*   **`bool isConnected() const`:**  
    Returns `true` if connected to Tello, `false` otherwise.

## TelloStatus Return Values
The `connect()` function and most command functions return a `TelloStatus` value:

*   **`TelloStatus::OK`:**  
    Command executed successfully.

*   **`TelloStatus::Timeout`:**  
    Command timed out waiting for response.

*   **`TelloStatus::ConnectionLost`:**  
    Connection to the drone was lost.

*   **`TelloStatus::NotConnected`:**  
    Command failed because drone is not connected.

*   **`TelloStatus::NoResponse`:**  
    No response received from drone.

*   **`TelloStatus::InvalidParameter`:**  
    Command failed due to invalid parameter value.

*   **`TelloStatus::UdpError`:**  
    UDP communication error occurred.

*   **`TelloStatus::UnknownError`:**  
    Unspecified error occurred.
