#include "ClawAxis.h"

static constexpr int      FREQ = 50;   
static constexpr int      RES  = 14;   
static constexpr uint32_t PERIOD_US = 20000;

// 500 ms covers full travel conservatively.
static constexpr unsigned int SERVO_TRAVEL_MS = 500;

ClawAxis::ClawAxis(int signalPin, int openUs, int clampUs)
    : _signalPin(signalPin),
      _openUs(openUs),
      _clampUs(clampUs)
{}

void ClawAxis::begin() {
    if(!ledcAttach(_signalPin, FREQ, RES)) {
        Serial.println("CLAW ERROR: LEDC Setup Failed!");
    }
    
    release();
}

void ClawAxis::clamp() {
    writeMicroseconds(_clampUs);
    delay(SERVO_TRAVEL_MS);
}

void ClawAxis::release() {
    writeMicroseconds(_openUs);
    delay(SERVO_TRAVEL_MS);
}

void ClawAxis::writeMicroseconds(int us) {
    // Duty = (microseconds / period) * (2^resolution - 1)
    uint32_t max_duty = (1 << RES) - 1;
    uint32_t duty = (uint32_t)(((uint64_t)us * max_duty) / PERIOD_US);
    
    ledcWrite(_signalPin, duty);
}