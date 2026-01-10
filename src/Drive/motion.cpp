#include "motion.h"
#include "Pid.h"
#include "main.h"
#include <algorithm>
#include <cmath>

// Motors from main.cpp
extern pros::Motor Back_left, Front_left, back_right, front_right;
extern pros::Motor Top_back_Left, Top_front_Left, Top_back_Right, Top_front_Right;

// PID tuning from main.cpp (edit there!)
extern double MTP_DIST_KP, MTP_DIST_KI, MTP_DIST_KD;
extern double MTP_HEAD_KP, MTP_HEAD_KI, MTP_HEAD_KD, MTP_HEAD_MAX;
extern double TURN_KP, TURN_KI, TURN_KD;

static Odom* g_odom = nullptr;

static inline int clamp127(double v) {
  if (v > 127) return 127;
  if (v < -127) return -127;
  return (int)std::round(v);
}

static inline double wrap180(double deg) {
  while (deg > 180) deg -= 360;
  while (deg < -180) deg += 360;
  return deg;
}

void motion_init(Odom* odom) { g_odom = odom; }

// IMPORTANT: This mixing matches your driver control exactly.
void set_drive_mecanum(double forward, double strafe, double turn) {
  // Flip forward to match teleop sign convention (teleop negates Y stick)
  forward = -forward;
  // Note: turn is NOT flipped - teleop negates it, but PID already outputs correct sign
  
  // Match opcontrol mapping:
  // BL: y + x + turn
  // FL: y - x + turn
  // BR: y - x - turn
  // FR: y + x - turn
  double bl = forward + strafe + turn;
  double fl = forward - strafe + turn;
  double br = forward - strafe - turn;
  double fr = forward + strafe - turn;

  double maxMag = std::max({std::fabs(fl), std::fabs(bl), std::fabs(fr), std::fabs(br), 127.0});
  fl = fl * 127.0 / maxMag;
  bl = bl * 127.0 / maxMag;
  fr = fr * 127.0 / maxMag;
  br = br * 127.0 / maxMag;

  // Apply to your 8 motors (paired)
  Front_left.move(clamp127(fl));
  Top_front_Left.move(clamp127(fl));

  Back_left.move(clamp127(bl));
  Top_back_Left.move(clamp127(bl));

  front_right.move(clamp127(fr));
  Top_front_Right.move(clamp127(fr));

  back_right.move(clamp127(br));
  Top_back_Right.move(clamp127(br));
}

void turn_to_angle(double targetDeg, double max_speed, double timeoutMs) {
  if (!g_odom) return;

  PID turn(TURN_KP, TURN_KI, TURN_KD);
  turn.setOutputLimits(-max_speed, max_speed);
  turn.setIntegralLimits(-5000, 5000);

  uint32_t start = pros::millis();
  uint32_t last  = start;

  while (pros::millis() - start < (uint32_t)timeoutMs) {
    g_odom->update();

    double curDeg = g_odom->getHeadingDeg();
    double err = wrap180(targetDeg - curDeg);
    if (std::fabs(err) < 1.0) break;

    uint32_t now = pros::millis();
    double dt = (now - last) / 1000.0;
    last = now;

    double t = -turn.step(err, dt);  // Negate to match motor direction
    set_drive_mecanum(0, 0, t);

    pros::delay(10);
  }
  set_drive_mecanum(0, 0, 0);
}

void move_to_point(double targetX, double targetY, double max_speed, double timeoutMs) {
  if (!g_odom) return;

  PID dist(MTP_DIST_KP, MTP_DIST_KI, MTP_DIST_KD);
  PID head(MTP_HEAD_KP, MTP_HEAD_KI, MTP_HEAD_KD);
  dist.setOutputLimits(-max_speed, max_speed);
  head.setOutputLimits(-MTP_HEAD_MAX, MTP_HEAD_MAX);
  dist.setIntegralLimits(-5000, 5000);
  head.setIntegralLimits(-5000, 5000);

  uint32_t start = pros::millis();
  uint32_t last  = start;

  while (pros::millis() - start < (uint32_t)timeoutMs) {
    g_odom->update();
    Pose2D p = g_odom->getPose();

    double dx = targetX - p.x;
    double dy = targetY - p.y;
    double d = std::sqrt(dx*dx + dy*dy);
    if (d < 0.75) break;

    // Field -> robot frame (your convention: X forward, Y right)
    double c = std::cos(p.theta);
    double s = std::sin(p.theta);
    double forwardErr =  c * dx + s * dy;
    double strafeErr  = -s * dx + c * dy;

    // Normalize to a direction
    double mag = std::max(1.0, std::sqrt(forwardErr*forwardErr + strafeErr*strafeErr));
    double dirF = forwardErr / mag;
    double dirS = strafeErr  / mag;

    uint32_t now = pros::millis();
    double dt = (now - last) / 1000.0;
    last = now;

    double speed = dist.step(d, dt);
    speed = std::min(speed, max_speed);

    // Slow down near target
    if (d < 6.0) speed = std::min(speed, 45.0);

    double forward = dirF * speed;
    double strafe  = dirS * speed;

    // Face the point while moving
    double targetDeg = std::atan2(dy, dx) * 180.0 / M_PI;
    double curDeg = g_odom->getHeadingDeg();
    double headingErr = wrap180(targetDeg - curDeg);
    double turn = -head.step(headingErr, dt);  // Negate to match motor direction

    set_drive_mecanum(forward, strafe, turn);
    pros::delay(10);
  }

  set_drive_mecanum(0, 0, 0);
}

void drive_distance(double inches, double max_speed, double timeoutMs) {
  if (!g_odom) return;

  g_odom->update();
  Pose2D p = g_odom->getPose();

  double tx = p.x + inches * std::cos(p.theta);
  double ty = p.y + inches * std::sin(p.theta);

  move_to_point(tx, ty, max_speed, timeoutMs);
}

void drive_straight(double inches, double max_speed, double timeoutMs) {
  if (!g_odom) return;

  g_odom->update();
  Pose2D start = g_odom->getPose();
  double targetHeading = g_odom->getHeadingDeg();  // Lock current heading

  PID dist(MTP_DIST_KP, MTP_DIST_KI, MTP_DIST_KD);
  PID head(MTP_HEAD_KP, MTP_HEAD_KI, MTP_HEAD_KD);
  dist.setOutputLimits(-max_speed, max_speed);
  head.setOutputLimits(-MTP_HEAD_MAX, MTP_HEAD_MAX);

  uint32_t startTime = pros::millis();
  uint32_t last = startTime;

  while (pros::millis() - startTime < (uint32_t)timeoutMs) {
    g_odom->update();
    Pose2D p = g_odom->getPose();

    // Distance traveled along starting heading direction
    double dx = p.x - start.x;
    double dy = p.y - start.y;
    double traveled = dx * std::cos(start.theta) + dy * std::sin(start.theta);
    double remaining = inches - traveled;

    if (std::fabs(remaining) < 0.5) break;

    uint32_t now = pros::millis();
    double dt = (now - last) / 1000.0;
    last = now;

    double forward = dist.step(remaining, dt);
    forward = std::clamp(forward, -max_speed, max_speed);

    // Slow near target
    if (std::fabs(remaining) < 6.0) forward = std::clamp(forward, -45.0, 45.0);

    // Maintain locked heading (not face target)
    double curDeg = g_odom->getHeadingDeg();
    double headingErr = wrap180(targetHeading - curDeg);
    double turn = -head.step(headingErr, dt);

    set_drive_mecanum(forward, 0, turn);
    pros::delay(10);
  }

  set_drive_mecanum(0, 0, 0);
}
