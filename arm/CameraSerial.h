#pragma once
#include <Arduino.h>

// Receives XY coordinate packets from the ESP32 camera module over UART.
// Packet format: 8 bytes — float x (bytes 0–3), float y (bytes 4–7), little-endian.
class CameraSerial {
public:
    // uartNum: HardwareSerial port index (e.g. 1 for Serial1)
    // rxPin / txPin: ESP32 GPIO pins
    // baud: UART baud rate (must match camera firmware)
    CameraSerial(int uartNum, int rxPin, int txPin, uint32_t baud = 115200);

    void begin();

    // Sends a request byte to the camera to trigger a capture
    void requestCoord();

    // Returns true and populates x/y if a complete 8-byte packet is available.
    // Returns false if fewer than 8 bytes are waiting.
    bool readCoord(float &x, float &y);

    // Pass-through: forwards any received bytes to Serial (for debugging).
    void passthrough();

private:
    static constexpr int PACKET_SIZE = 8;
    static constexpr char REQ_COMMAND = 'R'; // Command character

    HardwareSerial _serial;
    int   _rxPin;
    int   _txPin;
    uint32_t _baud;
};