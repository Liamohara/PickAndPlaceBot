#include "Controller.h"
#include <numbers>

using std::numbers::pi;

Controller::Controller()
    : _kin(L1, L2, L3, D),
      _axis1(PUL_PINS[0], DIR_PINS[0], 1600, 8, 0.0f, MIN_DEG[0], MAX_DEG[0]),
      _axis2(PUL_PINS[1], DIR_PINS[1], 1600, 4, 0.0f, MIN_DEG[1], MAX_DEG[1]),
      _axis3(PUL_PINS[2], DIR_PINS[2], 1600, 4, 0.0f, MIN_DEG[2], MAX_DEG[2]),
      _axes{&_axis1, &_axis2, &_axis3},
      _claw(CLAW_PIN),
      _camera(CAM_UART, CAM_RX, CAM_TX)
{}

void Controller::begin() {
    _claw.begin();
    _camera.begin();
    _engine.init();
    for (int i = 0; i < 3; i++) {
        _axes[i]->begin(_engine);
    }
}

bool Controller::pick() {
    release();
    if (!moveToCartesian({ -TRAVEL_R * sin(-pi/3) , -TRAVEL_R * cos(-pi/3), TRAVEL_Z })) return false;
    waitDone();
 
    delay(1000);

    _camera.requestCoord(); // Demand a new reading

    float x, y;
    unsigned long startTime = millis();
    // Loop until data arrives or a 5-second timeout occurs
    while (!_camera.readCoord(x, y)) {
        if (millis() - startTime > 5000) {
            Serial.println("Camera timeout!");
            return false;
        }
        delay(10); 
    }

    Serial.printf("Picking at X:%.2f Y:%.2f\n", x, y);
 
    float bearing = sqrt(x*x + y*y);
    float tx = (x / bearing) * TRAVEL_R;
    float ty = (y / bearing) * TRAVEL_R;
    CartesianCoord travel = { tx, ty, TRAVEL_Z };
 
    if (!moveToCartesian(travel)) return false;
    waitDone();
 
    if (!moveToCartesian({x, y, PICK_Z})) return false;
    waitDone();

    clamp();
    delay(1000);

    if (!moveToCartesian(travel)) return false;
    waitDone();
 
    return true;
}

bool Controller::place() {
    Serial.println("Placing disc...");
    moveToJoints(PLACE_ANGLES);
    waitDone();
    release();
    waitDone();
    return true;
}

void Controller::home() {
    Serial.println("Homing...");
    JointCoord zero = { 0.0f, 0.0f, 0.0f };
    moveToJoints(zero);
    waitDone();
}

// --- Arm motion ---

bool Controller::moveAxis(int axis, float angleDeg) {
    if (axis < 1 || axis > 3) return false;
    return _axes[axis - 1]->moveToAngle(angleDeg);
}

void Controller::moveToJoints(const JointCoord &q) {
    _axes[0]->moveToAngle(q.theta1);
    _axes[1]->moveToAngle(q.theta2);
    _axes[2]->moveToAngle(q.theta3);
}

bool Controller::moveToCartesian(const CartesianCoord &p) {
    Serial.print("Cartesian:\t"); Serial.print(p.x); Serial.print("\t"); Serial.print(p.y); Serial.print("\t"); Serial.println(p.z);
    JointCoord qRad;
    if (!_kin.inverseKinematics(p, qRad)) return false;

    JointCoord qDeg;
    qDeg.theta1 = qRad.theta1 * (180.0f / pi);
    qDeg.theta2 = qRad.theta2 * (180.0f / pi);
    qDeg.theta3 = qRad.theta3 * (180.0f / pi);

    Serial.print("Joint Angles:\t"); Serial.print(qDeg.theta1); Serial.print("\t"); Serial.print(qDeg.theta2); Serial.print("\t"); Serial.println(qDeg.theta3);
    Serial.print("Joint Angles:\t"); Serial.print(qRad.theta1); Serial.print("\t"); Serial.print(qRad.theta2); Serial.print("\t"); Serial.println(qRad.theta3);

    moveToJoints(qDeg);
    return true;
}

void Controller::waitDone() {
    for (int i = 0; i < 3; i++) _axes[i]->waitDone();
}

// --- Claw ---

void Controller::clamp() { _claw.clamp(); }
void Controller::release() { _claw.release(); }