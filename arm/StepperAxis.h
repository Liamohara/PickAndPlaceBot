#pragma once
#include <Arduino.h>
#include <FastAccelStepper.h>

// DM556T axis — PUL+ and DIR+ driven directly from ESP32 (single-ended).
// PUL-/DIR-/ENA tied to GND on the driver. ENA left unconnected = always enabled.
class StepperAxis {
public:
    StepperAxis(int   pulPin,
                int   dirPin,
                int   stepPerRev,
                int   gearRatio,
                float initialAngleDeg,
                float minAngleDeg,
                float maxAngleDeg);

    // Must be called after FastAccelStepperEngine::init()
    void begin(FastAccelStepperEngine &engine);

    // Non-blocking: queues the move. Returns false if target is outside limits.
    bool moveToAngle(float targetAngleDeg, float maxStepRateHz = 1500.0f);

    // Returns true while the stepper is still moving
    bool isRunning() const;

    // Block until this axis finishes its current move
    void waitDone() const;

private:
    long angleToSteps(float angleDeg) const;

    int   _pulPin;
    int   _dirPin;
    int   _stepPerRev;
    int   _gearRatio;
    float _minAngleDeg;
    float _maxAngleDeg;
    long  _currSteps;

    FastAccelStepper *_stepper = nullptr;
};