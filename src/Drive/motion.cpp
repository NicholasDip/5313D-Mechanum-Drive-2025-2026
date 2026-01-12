#include "motion.h"
#include "Pid.h"
#include "main.h"
#include <cmath>
#include <algorithm>

// ============================================================================
// EXTERNAL REFERENCES
// ============================================================================

// Motors from main.cpp (8-motor mecanum, but we drive as tank for auton)
extern pros::Motor Back_left, Front_left, back_right, front_right;
extern pros::Motor Top_back_Left, Top_front_Left, Top_back_Right, Top_front_Right;

// PID tuning from main.cpp
extern double MTP_DIST_KP, MTP_DIST_KI, MTP_DIST_KD;
extern double MTP_HEAD_KP, MTP_HEAD_KI, MTP_HEAD_KD, MTP_HEAD_MAX;
extern double TURN_KP, TURN_KI, TURN_KD;

static Odom* g_odom = nullptr;

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

// Wrap angle to [-180, 180]
static double wrap180(double deg) {
    while (deg > 180) deg -= 360;
    while (deg < -180) deg += 360;
    return deg;
}

// Clamp value to [-127, 127]
static int clamp127(double v) {
    return (int)std::clamp(v, -127.0, 127.0);
}

// ============================================================================
// PUBLIC FUNCTIONS
// ============================================================================

void motion_init(Odom* odom) {
    g_odom = odom;
}

// Set drive motors (tank-style for auton - no strafing)
// forward: positive = forward, turn: positive = counter-clockwise
void set_drive(double forward, double turn) {
    double left = forward + turn;
    double right = forward - turn;

    // Normalize if over max
    double maxMag = std::max({std::fabs(left), std::fabs(right), 127.0});
    left = left * 127.0 / maxMag;
    right = right * 127.0 / maxMag;

    // Apply to all 8 motors (left side / right side)
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

    const double ERROR_THRESHOLD = 2.0;      // degrees - consider "arrived"
    const double SETTLE_THRESHOLD = 1.0;     // degrees - for settling detection
    const int SETTLE_TIME_REQUIRED = 100;    // ms robot must be settled

    uint32_t startTime = pros::millis();
    uint32_t lastTime = startTime;
    uint32_t settleStart = 0;
    bool settling = false;

    while (pros::millis() - startTime < (uint32_t)timeoutMs) {
        g_odom->update();

        double currentDeg = g_odom->getHeadingDeg();
        double error = wrap180(targetDeg - currentDeg);

        // Check if we're within settling threshold
        if (std::fabs(error) < SETTLE_THRESHOLD) {
            if (!settling) {
                settling = true;
                settleStart = pros::millis();
            } else if (pros::millis() - settleStart >= SETTLE_TIME_REQUIRED) {
                // Settled for long enough, we're done
                break;
            }
        } else {
            settling = false;
        }

        // Quick exit if very close
        if (std::fabs(error) < ERROR_THRESHOLD && !settling) {
            settling = true;
            settleStart = pros::millis();
        }

        uint32_t now = pros::millis();
        double dt = (now - lastTime) / 1000.0;
        lastTime = now;

        double turnPower = turnPID.step(error, dt);

        // Minimum power to overcome static friction (tune this for your robot)
        if (std::fabs(error) > ERROR_THRESHOLD && std::fabs(turnPower) < 15) {
            turnPower = (turnPower > 0) ? 15 : -15;
        }

        set_drive(0, turnPower);
        pros::delay(10);
    }

    stop_drive();
    pros::delay(20); // Brief pause after turning
}

// ============================================================================
// DRIVE STRAIGHT - Drive forward/backward while maintaining heading
// ============================================================================

void drive_straight(double inches, double maxSpeed, double timeoutMs) {
    if (!g_odom) return;

    g_odom->update();
    Pose2D start = g_odom->getPose();
    double targetHeading = g_odom->getHeadingDeg();

    PID drivePID(MTP_DIST_KP, MTP_DIST_KI, MTP_DIST_KD);
    PID headPID(MTP_HEAD_KP, MTP_HEAD_KI, MTP_HEAD_KD);
    drivePID.setOutputLimits(-maxSpeed, maxSpeed);
    headPID.setOutputLimits(-30, 30); // Limit heading correction

    const double DIST_THRESHOLD = 0.75; // inches
    const int SETTLE_TIME = 80;         // ms

    uint32_t startTime = pros::millis();
    uint32_t lastTime = startTime;
    uint32_t settleStart = 0;
    bool settling = false;

    // Determine direction
    double direction = (inches >= 0) ? 1.0 : -1.0;
    double targetDist = std::fabs(inches);

    while (pros::millis() - startTime < (uint32_t)timeoutMs) {
        g_odom->update();
        Pose2D current = g_odom->getPose();

        // Calculate distance traveled along starting heading
        double dx = current.x - start.x;
        double dy = current.y - start.y;
        double traveled = std::fabs(dx * std::cos(start.theta) + dy * std::sin(start.theta));
        double remaining = targetDist - traveled;

        // Check settling
        if (remaining < DIST_THRESHOLD) {
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

        // Drive power (positive = forward)
        double drivePower = drivePID.step(remaining, dt) * direction;

        // Slow down near target
        if (remaining < 6.0) {
            drivePower = std::clamp(drivePower, -40.0, 40.0);
        }

        // Heading correction
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

    // Calculate distance and angle to target
    double dx = targetX - start.x;
    double dy = targetY - start.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // If already at target (within 1 inch), just turn to final heading
    if (distance < 1.0) {
        turn_to_angle(endHeading, maxSpeed, timeoutMs);
        return;
    }

    // Calculate angle TO the target point (using atan2)
    // atan2(dy, dx) gives angle from +X axis
    // For standard VEX coords where 0° = +Y, we need: atan2(dx, dy)
    double angleToTarget = std::atan2(dx, dy) * 180.0 / M_PI;

    // Allocate time: ~30% turn, ~50% drive, ~20% final turn
    double turnTime = timeoutMs * 0.25;
    double driveTime = timeoutMs * 0.55;
    double finalTurnTime = timeoutMs * 0.20;

    // PHASE 1: Turn to face the target
    turn_to_angle(angleToTarget, maxSpeed, turnTime);

    // PHASE 2: Drive forward to the target with heading correction
    PID drivePID(MTP_DIST_KP, MTP_DIST_KI, MTP_DIST_KD);
    PID headPID(MTP_HEAD_KP, MTP_HEAD_KI, MTP_HEAD_KD);
    drivePID.setOutputLimits(-maxSpeed, maxSpeed);
    headPID.setOutputLimits(-25, 25);

    const double DIST_THRESHOLD = 1.0;
    const int SETTLE_TIME = 60;

    uint32_t startTime = pros::millis();
    uint32_t lastTime = startTime;
    uint32_t settleStart = 0;
    bool settling = false;

    while (pros::millis() - startTime < (uint32_t)driveTime) {
        g_odom->update();
        Pose2D current = g_odom->getPose();

        // Recalculate distance to target
        dx = targetX - current.x;
        dy = targetY - current.y;
        double remaining = std::sqrt(dx * dx + dy * dy);

        // Check if arrived
        if (remaining < DIST_THRESHOLD) {
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

        // Drive forward
        double drivePower = drivePID.step(remaining, dt);

        // Slow down when close
        if (remaining < 8.0) {
            drivePower = std::clamp(drivePower, -50.0, 50.0);
        }
        if (remaining < 4.0) {
            drivePower = std::clamp(drivePower, -35.0, 35.0);
        }

        // Correct heading to stay pointed at target
        double newAngleToTarget = std::atan2(dx, dy) * 180.0 / M_PI;
        double headingError = wrap180(newAngleToTarget - g_odom->getHeadingDeg());
        double turnPower = headPID.step(headingError, dt);

        set_drive(drivePower, turnPower);
        pros::delay(10);
    }

    stop_drive();
    pros::delay(20);

    // PHASE 3: Turn to final heading
    turn_to_angle(endHeading, maxSpeed * 0.8, finalTurnTime);
}

// ============================================================================
// PATH FOLLOWING
// ============================================================================

void follow_path(const Waypoint* path, int numPoints) {
    if (!g_odom || !path || numPoints <= 0) return;

    for (int i = 0; i < numPoints; i++) {
        move_to_point(
            path[i].x,
            path[i].y,
            path[i].heading,
            path[i].speed,
            3000  // 3 second timeout per point
        );
    }
}
