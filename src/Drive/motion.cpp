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
// Negate forward to match physical robot direction
void set_drive(double forward, double turn) {
    forward = -forward;  // Invert forward direction
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
// ============================================================================

void turn_to_angle(double targetDeg, double maxSpeed, double timeoutMs) {
    if (!g_odom) return;

    PID turnPID(TURN_KP, TURN_KI, TURN_KD);
    turnPID.setOutputLimits(-maxSpeed, maxSpeed);
    turnPID.setIntegralLimits(-3000, 3000);

    const double ERROR_THRESHOLD = 2.0;
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
        if (std::fabs(error) > ERROR_THRESHOLD && std::fabs(turnPower) < 15) {
            turnPower = (turnPower > 0) ? 15 : -15;
        }

        set_drive(0, turnPower);
        pros::delay(10);
    }

    stop_drive();
    pros::delay(20);
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
// MOVE TO POINT - Turn-then-drive approach
// ============================================================================

void move_to_point(double targetX, double targetY, double endHeading, double maxSpeed, double timeoutMs) {
    if (!g_odom) return;

    g_odom->update();
    Pose2D start = g_odom->getPose();

    double dx = targetX - start.x;
    double dy = targetY - start.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // If already at target, just turn to final heading
    if (distance < 1.0) {
        turn_to_angle(endHeading, maxSpeed, timeoutMs);
        return;
    }

    // Calculate angle TO the target point
    // atan2(dx, dy) gives angle from +Y axis (0° = forward)
    double angleToTarget = std::atan2(dx, dy) * 180.0 / M_PI;

    // Time allocation
    double turnTime = timeoutMs * 0.25;
    double driveTime = timeoutMs * 0.55;
    double finalTurnTime = timeoutMs * 0.20;

    // PHASE 1: Turn to face the target
    turn_to_angle(angleToTarget, maxSpeed, turnTime);

    // PHASE 2: Drive to the target
    PID drivePID(MTP_DIST_KP, MTP_DIST_KI, MTP_DIST_KD);
    PID headPID(MTP_HEAD_KP, MTP_HEAD_KI, MTP_HEAD_KD);
    drivePID.setOutputLimits(-maxSpeed, maxSpeed);
    headPID.setOutputLimits(-25, 25);

    uint32_t startTime = pros::millis();
    uint32_t lastTime = startTime;

    while (pros::millis() - startTime < (uint32_t)driveTime) {
        g_odom->update();
        Pose2D current = g_odom->getPose();

        dx = targetX - current.x;
        dy = targetY - current.y;
        double remaining = std::sqrt(dx * dx + dy * dy);

        if (remaining < 1.0) break;

        uint32_t now = pros::millis();
        double dt = (now - lastTime) / 1000.0;
        lastTime = now;

        double drivePower = drivePID.step(remaining, dt);
        if (remaining < 8.0) drivePower = std::clamp(drivePower, -50.0, 50.0);
        if (remaining < 4.0) drivePower = std::clamp(drivePower, -35.0, 35.0);

        // Keep pointing at target while driving
        double newAngle = std::atan2(dx, dy) * 180.0 / M_PI;
        double headingError = wrap180(newAngle - g_odom->getHeadingDeg());
        double turnPower = headPID.step(headingError, dt);

        set_drive(drivePower, turnPower);
        pros::delay(10);
    }

    stop_drive();
    pros::delay(20);

    // PHASE 3: Turn to final heading
    turn_to_angle(endHeading, maxSpeed * 0.8, finalTurnTime);
}
