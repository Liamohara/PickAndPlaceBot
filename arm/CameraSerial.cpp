#include "CameraSerial.h"

CameraSerial::CameraSerial(int uartNum, int rxPin, int txPin, uint32_t baud)
    : _serial(uartNum), _rxPin(rxPin), _txPin(txPin), _baud(baud) {}

void CameraSerial::begin() {
    _serial.begin(_baud, SERIAL_8N1, _rxPin, _txPin);
}

void CameraSerial::requestCoord() {
    _serial.write(REQ_COMMAND); // Send the request trigger
}

bool CameraSerial::readCoord(float &x, float &y) {
    // Wait for the specific packet size
    if (_serial.available() < PACKET_SIZE) return false;

    uint8_t buf[PACKET_SIZE];
    _serial.readBytes(buf, PACKET_SIZE);
    memcpy(&x, &buf[0], 4);
    memcpy(&y, &buf[4], 4);
    return true;
}

void CameraSerial::passthrough() {
    while (_serial.available()) {
        Serial.write(_serial.read());
    }
}