#include "Kinematics.h"
#include <numbers>
#include <algorithm>

using std::numbers::pi;
using std::clamp;

ArmKinematics::ArmKinematics(float l1, float l2, float l3, float d)
    : L1(l1), L2(l2), L3(l3), D(d) {}

CartesianCoord ArmKinematics::forwardKinematics(const JointCoord &q) const {
    CartesianCoord p;
    float gamma = sin(q.theta2) * L2 + sin(q.theta2 + q.theta3) * L3;
    p.x = -cos(q.theta1) * gamma + D * sin(q.theta1);
    p.y = -sin(q.theta1) * gamma - D * cos(q.theta1);
    p.z = L1 + cos(q.theta2) * L2 + cos(q.theta2 + q.theta3) * L3;
    return p;
}

bool ArmKinematics::inverseKinematics(const CartesianCoord &p, JointCoord &q) const {
    q.theta1 = atan2(p.y, p.x);

    float dx = p.x;
    float dy = p.y;
    float dz = p.z - L1;

    float r2 = dx*dx + dy*dy + dz*dz;
    float r  = sqrt(r2);

    float lEE = sqrt(L3*L3 + D*D);

    // Reject targets that are genuinely out of reach before calling acos.
    float reach = L2 + lEE;
    float diff  = fabs(L2 - lEE);
    if (r > reach || r < diff) return false;

    // Clamp cosine values to [-1, 1] before passing to acos.
    // Floating point arithmetic can produce values like 1.0000001 for targets
    // exactly on the workspace boundary, which acos turns into NaN.
    float cosVal2 = (L2*L2 + r2 - lEE*lEE) / (2.0f * L2 * r);
    float sigma2  = acos(clamp(cosVal2, -1.0f, 1.0f));
    // sigma2 = -sigma2;  // uncomment for alternate (elbow-down) solution
    q.theta2 = pi / 2 - atan2(dz, sqrt(dx*dx + dy*dy)) - sigma2;

    float cosVal3 = (L2*L2 + lEE*lEE - r2) / (2.0f * L2 * lEE);
    float sigma3  = acos(clamp(cosVal3, -1.0f, 1.0f));
    // sigma3 = -sigma3;  // uncomment for alternate (elbow-down) solution
    q.theta3 = sigma3 + atan2(D, L3) - pi;

    return true;
}