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

// Strafe compensation from main.cpp
extern double STRAFE_MULTIPLIER;
extern double MIN_STRAFE_POWER;

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

// ============================================================================
// HOLONOMIC DRIVE - Forward, Strafe, and Turn
// ============================================================================
void set_drive(double forward, double strafe, double turn) {
    // Apply strafe friction compensation
    if (std::fabs(strafe) > 5.0) {
        // Apply minimum power to overcome static friction
        if (std::fabs(strafe) < MIN_STRAFE_POWER) {
            strafe = (strafe > 0) ? MIN_STRAFE_POWER : -MIN_STRAFE_POWER;
        }
        // Boost strafe to compensate for roller friction
        strafe = strafe * STRAFE_MULTIPLIER;
    }

    // Invert forward to match physical robot direction
    forward = -forward;

    // Mecanum wheel mixing:
    // FL = forward - strafe + turn
    // FR = forward + strafe - turn
    // BL = forward + strafe + turn
    // BR = forward - strafe - turn
    double fl = forward - strafe + turn;
    double fr = forward + strafe - turn;
    double bl = forward + strafe + turn;
    double br = forward - strafe - turn;

    // Normalize if any motor exceeds 127
    double maxMag = std::max({std::fabs(fl), std::fabs(fr), std::fabs(bl), std::fabs(br), 127.0});
    if (maxMag > 127) {
        fl = fl * 127.0 / maxMag;
        fr = fr * 127.0 / maxMag;
        bl = bl * 127.0 / maxMag;
        br = br * 127.0 / maxMag;
    }

    // Apply to all 8 motors (4 main + 4 top)
    Front_left.move(clamp127(fl));
    Top_front_Left.move(clamp127(fl));
    Back_left.move(clamp127(bl));
    Top_back_Left.move(clamp127(bl));

    front_right.move(clamp127(fr));
    Top_front_Right.move(clamp127(fr));
    back_right.move(clamp127(br));
    Top_back_Right.move(clamp127(br));
}

void stop_drive() {
    set_drive(0, 0, 0);
}

// ============================================================================
// TURN TO ANGLE - In-place rotation with settling
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

        set_drive(0, 0, turnPower);
        pros::delay(10);
    }

    stop_drive();
}

// ============================================================================
// DRIVE STRAIGHT - Maintain heading while driving forward/backward
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

        set_drive(drivePower, 0, turnPower);
        pros::delay(10);
    }

    stop_drive();
}

// ============================================================================
// STRAFE - Move left/right while maintaining heading
// Positive inches = right, Negative inches = left
// ============================================================================
void strafe(double inches, double maxSpeed, double timeoutMs) {
    if (!g_odom) return;

    g_odom->update();
    Pose2D start = g_odom->getPose();
    double targetHeading = g_odom->getHeadingDeg();

    PID strafePID(MTP_DIST_KP, MTP_DIST_KI, MTP_DIST_KD);
    PID headPID(MTP_HEAD_KP, MTP_HEAD_KI, MTP_HEAD_KD);
    strafePID.setOutputLimits(-maxSpeed, maxSpeed);
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

        double strafePower = strafePID.step(remaining, dt) * direction;
        if (remaining < 6.0) strafePower = std::clamp(strafePower, -40.0, 40.0);

        double headingError = wrap180(targetHeading - g_odom->getHeadingDeg());
        double turnPower = headPID.step(headingError, dt);

        set_drive(0, strafePower, turnPower);
        pros::delay(10);
    }

    stop_drive();
}

// ============================================================================
// MOVE TO POINT - Holonomic movement to any field position
// Coordinate System:
//   +X = forward, +Y = right
//   Heading 0° = facing +X, clockwise positive
// ============================================================================
void move_to_point(double targetX, double targetY, double endHeading, double maxSpeed, double timeoutMs) {
    if (!g_odom) return;

    PID xPID(MTP_DIST_KP, MTP_DIST_KI, MTP_DIST_KD);
    PID yPID(MTP_DIST_KP, MTP_DIST_KI, MTP_DIST_KD);
    PID headPID(MTP_HEAD_KP, MTP_HEAD_KI, MTP_HEAD_KD);
    
    xPID.setOutputLimits(-maxSpeed, maxSpeed);
    yPID.setOutputLimits(-maxSpeed, maxSpeed);
    headPID.setOutputLimits(-MTP_HEAD_MAX, MTP_HEAD_MAX);

    const double EXIT_TOLERANCE = 2.0;  // inches

    uint32_t startTime = pros::millis();
    uint32_t lastTime = startTime;

    while (pros::millis() - startTime < (uint32_t)timeoutMs) {
        g_odom->update();
        Pose2D current = g_odom->getPose();

        // Field-frame error (where we need to go)
        double errorX = targetX - current.x;  // + means need to go forward in field
        double errorY = targetY - current.y;  // + means need to go right in field
        double distance = std::sqrt(errorX * errorX + errorY * errorY);

        // Exit when close enough
        if (distance < EXIT_TOLERANCE) break;

        // Convert field error to robot-relative error
        // Robot heading in radians
        double headingRad = current.theta;
        double cosH = std::cos(headingRad);
        double sinH = std::sin(headingRad);

        // Rotate field error into robot frame
        // If robot is facing +X (heading=0), then:
        //   robot forward = field +X
        //   robot right = field +Y
        // As robot rotates, we need to transform the error
        double robotForward = errorX * cosH + errorY * sinH;   // How much to drive forward
        double robotStrafe = -errorX * sinH + errorY * cosH;   // How much to strafe right

        // Heading error
        double headingError = wrap180(endHeading - g_odom->getHeadingDeg());

        uint32_t now = pros::millis();
        double dt = (now - lastTime) / 1000.0;
        lastTime = now;

        // PID outputs
        double forwardPower = xPID.step(robotForward, dt);
        double strafePower = yPID.step(robotStrafe, dt);
        double turnPower = headPID.step(headingError, dt);

        // Limit total drive power while preserving direction
        double driveMag = std::sqrt(forwardPower * forwardPower + strafePower * strafePower);
        if (driveMag > maxSpeed) {
            forwardPower = forwardPower * maxSpeed / driveMag;
            strafePower = strafePower * maxSpeed / driveMag;
        }

        set_drive(forwardPower, strafePower, turnPower);
        pros::delay(10);
    }

    stop_drive();
}
