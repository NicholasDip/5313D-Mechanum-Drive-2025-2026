#include "motion.h"
#include "Pid.h"
#include "main.h"
#include <cmath>
#include <algorithm>

// Motors from main.cpp
extern pros::Motor Back_left, Front_left, back_right, front_right;
extern pros::Motor Top_back_Left, Top_front_Left, Top_back_Right, Top_front_Right;

// PID tuning from main.cpp
extern double MTP_DIST_KP, MTP_DIST_KI, MTP_DIST_KD;
extern double MTP_HEAD_KP, MTP_HEAD_KI, MTP_HEAD_KD, MTP_HEAD_MAX;
extern double TURN_KP, TURN_KI, TURN_KD;

static Odom* g_odom = nullptr;

// Wrap angle to [-180, 180]
static double wrap180(double deg) {
    while (deg > 180) deg -= 360;
    while (deg < -180) deg += 360;
    return deg;
}

static int clamp127(double v) {
    if (v > 127) return 127;
    if (v < -127) return -127;
    return (int)v;
}

void motion_init(Odom* odom) {
    g_odom = odom;
}

// Tank-style drive (no strafe)
void set_drive(double forward, double turn) {
    forward = -forward;  // Invert forward direction to match physical robot
    turn = -turn;        // Invert turn direction to match physical robot
    double left = forward + turn;
    double right = forward - turn;

    double maxMag = std::max({std::fabs(left), std::fabs(right), 127.0});
    left = left * 127.0 / maxMag;
    right = right * 127.0 / maxMag;

    Front_left.move(clamp127(left));
    Top_front_Left.move(clamp127(left));
    Back_left.move(clamp127(left));
    Top_back_Left.move(clamp127(left));

    front_right.move(clamp127(right));
    Top_front_Right.move(clamp127(right));
    back_right.move(clamp127(right));
    Top_back_Right.move(clamp127(right));
}

void stop_drive() {
    set_drive(0, 0);
}

// ============================================================================
// TURN TO ANGLE - In-place rotation with settling
// VEX Standard: 0° = +Y, 90° = +X, clockwise positive
// ============================================================================
void turn_to_angle(double targetDeg, double maxSpeed, double timeoutMs) {
    if (!g_odom) return;

    PID turnPID(TURN_KP, TURN_KI, TURN_KD);
    turnPID.setOutputLimits(-maxSpeed, maxSpeed);
    turnPID.setIntegralLimits(-3000, 3000);

    const double SETTLE_THRESHOLD = 1.5;
    const int SETTLE_TIME = 100;

    uint32_t startTime = pros::millis();
    uint32_t lastTime = startTime;
    uint32_t settleStart = 0;
    bool settling = false;

    while (pros::millis() - startTime < (uint32_t)timeoutMs) {
        g_odom->update();

        double currentDeg = g_odom->getHeadingDeg();
        double error = wrap180(targetDeg - currentDeg);

        // Check settling
        if (std::fabs(error) < SETTLE_THRESHOLD) {
            if (!settling) {
                settling = true;
                settleStart = pros::millis();
            } else if (pros::millis() - settleStart >= SETTLE_TIME) {
                break;
            }
        } else {
            settling = false;
        }

        uint32_t now = pros::millis();
        double dt = (now - lastTime) / 1000.0;
        lastTime = now;

        double turnPower = turnPID.step(error, dt);

        // Min power to overcome friction
        if (std::fabs(error) > 2.0 && std::fabs(turnPower) < 15) {
            turnPower = (turnPower > 0) ? 15 : -15;
        }

        set_drive(0, turnPower);
        pros::delay(10);
    }

    stop_drive();
}

// ============================================================================
// DRIVE STRAIGHT - Maintain heading while driving
// ============================================================================
void drive_straight(double inches, double maxSpeed, double timeoutMs) {
    if (!g_odom) return;

    g_odom->update();
    Pose2D start = g_odom->getPose();
    double targetHeading = g_odom->getHeadingDeg();

    PID drivePID(MTP_DIST_KP, MTP_DIST_KI, MTP_DIST_KD);
    PID headPID(MTP_HEAD_KP, MTP_HEAD_KI, MTP_HEAD_KD);
    drivePID.setOutputLimits(-maxSpeed, maxSpeed);
    headPID.setOutputLimits(-30, 30);

    double direction = (inches >= 0) ? 1.0 : -1.0;
    double targetDist = std::fabs(inches);

    uint32_t startTime = pros::millis();
    uint32_t lastTime = startTime;

    while (pros::millis() - startTime < (uint32_t)timeoutMs) {
        g_odom->update();
        Pose2D current = g_odom->getPose();

        double dx = current.x - start.x;
        double dy = current.y - start.y;
        double traveled = std::sqrt(dx * dx + dy * dy);
        double remaining = targetDist - traveled;

        if (remaining < 0.75) break;

        uint32_t now = pros::millis();
        double dt = (now - lastTime) / 1000.0;
        lastTime = now;

        double drivePower = drivePID.step(remaining, dt) * direction;
        if (remaining < 6.0) drivePower = std::clamp(drivePower, -40.0, 40.0);

        double headingError = wrap180(targetHeading - g_odom->getHeadingDeg());
        double turnPower = headPID.step(headingError, dt);

        set_drive(drivePower, turnPower);
        pros::delay(10);
    }

    stop_drive();
}

// ============================================================================
// MOVE TO POINT - Seamless (no stopping between points)
// VEX Standard: 0° = +Y, 90° = +X, clockwise positive
// ============================================================================
void move_to_point(double targetX, double targetY, double endHeading, double maxSpeed, double timeoutMs) {
    if (!g_odom) return;

    PID distPID(MTP_DIST_KP, MTP_DIST_KI, MTP_DIST_KD);
    PID headPID(MTP_HEAD_KP, MTP_HEAD_KI, MTP_HEAD_KD);
    distPID.setOutputLimits(-maxSpeed, maxSpeed);
    headPID.setOutputLimits(-MTP_HEAD_MAX, MTP_HEAD_MAX);

    const double EXIT_TOLERANCE = 3.0;  // inches - exit when this close

    uint32_t startTime = pros::millis();
    uint32_t lastTime = startTime;

    while (pros::millis() - startTime < (uint32_t)timeoutMs) {
        g_odom->update();
        Pose2D current = g_odom->getPose();

        double dx = targetX - current.x;
        double dy = targetY - current.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Exit when close enough - no stopping, seamless transition
        if (distance < EXIT_TOLERANCE) break;

        // Angle to target for convention: +X forward, +Y right, CW positive
        // atan2(dy, dx) gives 0° when target is in +X direction (forward)
        double angleToTarget = std::atan2(dy, dx) * 180.0 / M_PI;

        double headingError = wrap180(angleToTarget - g_odom->getHeadingDeg());

        uint32_t now = pros::millis();
        double dt = (now - lastTime) / 1000.0;
        lastTime = now;

        double forward = distPID.step(distance, dt);
        double turn = headPID.step(headingError, dt);

        // Reduce forward power when not facing target
        double headingScale = std::cos(headingError * M_PI / 180.0);
        headingScale = std::max(headingScale, 0.0);
        forward *= headingScale;

        // Reduce turn correction when close (angle gets unstable near target)
        if (distance < 4.0) {
            turn *= (distance / 4.0);  // Linearly reduce turn as we approach
        }

        set_drive(forward, turn);
        pros::delay(10);
    }

    // No stop - seamless transition to next move_to_point
    // endHeading param unused in seamless mode
}