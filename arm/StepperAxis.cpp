#include "StepperAxis.h"
#include <cmath>

StepperAxis::StepperAxis(int pulPin, int dirPin, int stepPerRev, int gearRatio,
                         float initialAngleDeg, float minAngleDeg, float maxAngleDeg)
    : _pulPin(pulPin),
      _dirPin(dirPin),
      _stepPerRev(stepPerRev),
      _gearRatio(gearRatio),
      _minAngleDeg(minAngleDeg),
      _maxAngleDeg(maxAngleDeg),
      _currSteps(0)
{
    _currSteps = angleToSteps(initialAngleDeg);
}

void StepperAxis::begin(FastAccelStepperEngine &engine) {
    _stepper = engine.stepperConnectToPin(_pulPin);
    if (_stepper) {
        _stepper->setDirectionPin(_dirPin);
        _stepper->setCurrentPosition(_currSteps);
        // Default acceleration — keeps ramp to ~200 steps at 1500 Hz.
        _stepper->setAcceleration(5625);
    }
}

bool StepperAxis::moveToAngle(float targetAngleDeg, float maxStepRateHz) {
    if (!_stepper) return false;

    if (targetAngleDeg < _minAngleDeg || targetAngleDeg > _maxAngleDeg) return false;

    long targetSteps = angleToSteps(targetAngleDeg);
    long delta       = targetSteps - _currSteps;
    if (delta == 0) return true;

    uint32_t speedHz = (uint32_t)maxStepRateHz;
    _stepper->setSpeedInHz(speedHz);
    _stepper->setAcceleration((uint32_t)(speedHz * speedHz / 450UL));

    _stepper->moveTo(targetSteps);
    _currSteps = targetSteps;
    return true;
}

bool StepperAxis::isRunning() const {
    if (!_stepper) return false;
    return _stepper->isRunning();
}

void StepperAxis::waitDone() const {
    while (isRunning()) delay(1);
}

long StepperAxis::angleToSteps(float angleDeg) const {
    return std::lround(angleDeg / 360.0f * _stepPerRev * _gearRatio);
}