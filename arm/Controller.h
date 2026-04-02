#pragma once
#include <FastAccelStepper.h>
#include "Kinematics.h"
#include "StepperAxis.h"
#include "ClawAxis.h"
#include "CameraSerial.h"

class Controller {
public:
    Controller();

    void begin();

    // --- Arm motion ---

    // Single-axis move (1-indexed). Non-blocking.
    // Returns false if out of limits.
    bool moveAxis(int axis, float angleDeg);

    // Move all three joints simultaneously. Non-blocking.
    // q.theta1/2/3 in DEGREES.
    void moveToJoints(const JointCoord &q);

    // IK solve then move. Non-blocking.
    // Returns false if position is unreachable or any joint limit is exceeded.
    bool moveToCartesian(const CartesianCoord &p);

    // Block until all arm axes are stationary
    void waitDone();

    // --- Task Sequences ---

    // Reads XY from the camera, moves to a safe height above that point,
    // descends to PICK_Z, clamps, and retracts.
    // Returns false if no camera packet is available or the position is unreachable.
    bool pick();

    // Moves to a predefined drop-off location and releases the claw.
    bool place();

    // Returns all joints to 0,0,0.
    void home();

    // --- Claw ---

    // Move servo to gripping position. Blocking.
    void clamp();

    // Move servo to open position. Blocking.
    void release();

private:
    // Robot geometry (mm)
    static constexpr float L1 = 95.0f;
    static constexpr float L2 = 110.0f;
    static constexpr float L3 = 221.72f;
    static constexpr float D = 49.5f;

    // Travel waypoint: fixed height = L1, fixed radial distance from Z axis = TRAVEL_R.
    // The XY position of the waypoint is scaled along the target bearing so the
    // arm is always in the same folded configuration when traversing between points.
    static constexpr float TRAVEL_R = 210.0f;
    static constexpr float TRAVEL_Z = 120.0f;

    static constexpr float PICK_Z = 2.0f;

    // Fixed Place Location
    static constexpr JointCoord PLACE_ANGLES = {-104.0f, -20.0f, 110.0f};

    // Arm driver pins
    static constexpr int PUL_PINS[3] = { 40, 42, 1 };
    static constexpr int DIR_PINS[3] = { 39, 41, 2 };

    // Joint limits (degrees)
    static constexpr float MIN_DEG[3] = { -360.0f, -150.0f, -150.0f };
    static constexpr float MAX_DEG[3] = {  360.0f,  150.0f,  150.0f };

    // Servo signal pin
    static constexpr int CLAW_PIN = 14;

    // Camera UART
    static constexpr int CAM_UART = 1;
    static constexpr int CAM_RX   = 35;
    static constexpr int CAM_TX   = 36;

    FastAccelStepperEngine _engine;
    ArmKinematics          _kin;
    StepperAxis            _axis1;
    StepperAxis            _axis2;
    StepperAxis            _axis3;
    StepperAxis*           _axes[3];
    ClawAxis               _claw;
    CameraSerial           _camera;
};