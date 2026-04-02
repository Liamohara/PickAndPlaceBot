#pragma once
#include <Arduino.h>

// Claw actuator using a JP Supertec S3002 standard hobby servo.
// Wiring: 5V → servo red, GND → servo brown/black, control → CLAW_PIN.
// The claw does not affect kinematics (it is part of L3).
//
// Driven via ESP32 LEDC peripheral (Arduino core 3.x pin-based API).
// No servo library required — avoids MCPWM header conflicts with FastAccelStepper.
//
// Standard servo timing: 50 Hz frame, pulse width 1000–2000 µs.
// Adjust clampUs to grip your object without stalling the servo.
class ClawAxis {
public:
    // openUs:  pulse width in microseconds for fully open   (1000 µs = ~0°)
    // clampUs: pulse width in microseconds for grip position (tune to object)
    ClawAxis(int signalPin, int openUs = 900, int clampUs = 700);

    void begin();

    // Move to gripping position. Blocking.
    void clamp();

    // Move to fully open position. Blocking.
    void release();

private:
    void writeMicroseconds(int us);

    int _signalPin;
    int _openUs;
    int _clampUs;
};