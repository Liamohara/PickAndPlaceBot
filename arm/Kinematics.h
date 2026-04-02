#pragma once
#include <Arduino.h>

// Joint angles throughout the Kinematics layer are always in RADIANS.
// Controller is responsible for converting to degrees before commanding StepperAxis.
struct CartesianCoord { float x, y, z; };               // mm
struct JointCoord     { float theta1, theta2, theta3; }; // radians

class ArmKinematics {
public:
    ArmKinematics(float l1, float l2, float l3, float d);

    CartesianCoord forwardKinematics(const JointCoord &q) const;

    // Returns false if the target is unreachable or any acos domain check fails.
    bool inverseKinematics(const CartesianCoord &p, JointCoord &q) const;

    const float L1, L2, L3, D;
};
