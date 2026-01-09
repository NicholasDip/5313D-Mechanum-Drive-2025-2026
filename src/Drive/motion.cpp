
#include "Motion.h"
#include "main.h"
#include <cmath>
#include <algorithm>

// Motors defined in main.cpp
extern pros::Motor Back_left, Front_left, back_right, front_right;
extern pros::Motor Top_back_Left, Top_front_Left, Top_back_Right, Top_front_Right;

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

class PID {
public:
  PID(double p, double i, double d, double iLim = 1e9)
    : kP(p), kI(i), kD(d), iLimit(iLim) {}
  double step(double err) {
    integral += err;
    if (integral > iLimit) integral = iLimit;
    if (integral < -iLimit) integral = -iLimit;
    double deriv = err - prev;
    prev = err;
    return kP*err + kI*integral + kD*deriv;
  }
  void reset() { integral = 0; prev = 0; }
private:
  double kP, kI, kD;
  double integral = 0;
  double prev = 0;
  double iLimit;
};

void motion_init(Odom* odom) { g_odom = odom; }

// forward: + forward, strafe: + right, turn: + CCW
void set_drive_mecanum(double forward, double strafe, double turn) {
  double fl = forward + strafe + turn;
  double bl = forward - strafe + turn;
  double fr = forward - strafe - turn;
  double br = forward + strafe - turn;

  // scale preserve ratios
  double maxMag = std::max({std::fabs(fl), std::fabs(bl), std::fabs(fr), std::fabs(br), 127.0});
  fl = fl * 127.0 / maxMag;
  bl = bl * 127.0 / maxMag;
  fr = fr * 127.0 / maxMag;
  br = br * 127.0 / maxMag;

  Front_left.move(clamp127(fl));
  Top_front_Left.move(clamp127(fl));

  Back_left.move(clamp127(bl));
  Top_back_Left.move(clamp127(bl));

  front_right.move(clamp127(fr));
  Top_front_Right.move(clamp127(fr));

  back_right.move(clamp127(br));
  Top_back_Right.move(clamp127(br));
}

void turn_to_angle(double targetDeg, double maxSpeed, double timeoutMs) {
  PID turn(2.5, 0.01, 8.0, 5000);

  uint32_t start = pros::millis();
  while (pros::millis() - start < (uint32_t)timeoutMs) {
    g_odom->update();

    double curDeg = g_odom->getHeadingDeg();
    double err = wrap180(targetDeg - curDeg);
    if (std::fabs(err) < 1.0) break;

    double t = turn.step(err);
    t = std::clamp(t, -maxSpeed, maxSpeed);

    set_drive_mecanum(0, 0, t);
    pros::delay(10);
  }
  set_drive_mecanum(0, 0, 0);
}

void move_to_point(double targetX, double targetY, double maxSpeed, double timeoutMs) {
  PID dist(6.0, 0.0, 1.5, 5000);
  PID turn(2.0, 0.0, 6.0, 5000);

  uint32_t start = pros::millis();
  while (pros::millis() - start < (uint32_t)timeoutMs) {
    g_odom->update();

    Pose2D p = g_odom->getPose();
    double dx = targetX - p.x;
    double dy = targetY - p.y;
    double d = std::sqrt(dx*dx + dy*dy);
    if (d < 0.75) break;

    // Field -> robot frame (forward/right)
    double th = p.theta;
    double c = std::cos(th);
    double s = std::sin(th);
    double forwardErr =  c * dx + s * dy;
    double strafeErr  = -s * dx + c * dy;

    double mag = std::max(1.0, std::sqrt(forwardErr*forwardErr + strafeErr*strafeErr));
    double dirF = forwardErr / mag;
    double dirS = strafeErr  / mag;

    double speed = dist.step(d);
    speed = std::min(speed, maxSpeed);

    // optional slow near target
    if (d < 6.0) speed = std::min(speed, 50.0);

    double forward = dirF * speed;
    double strafe  = dirS * speed;

    // Face target
    double targetDeg = std::atan2(dy, dx) * 180.0 / M_PI;
    double curDeg = g_odom->getHeadingDeg();
    double headingErr = wrap180(targetDeg - curDeg);

    double t = turn.step(headingErr);
    t = std::clamp(t, -maxSpeed, maxSpeed);

    set_drive_mecanum(forward, strafe, t);
    pros::delay(10);
  }

  set_drive_mecanum(0, 0, 0);
}

void drive_distance(double inches, double maxSpeed, double timeoutMs) {
  g_odom->update();
  Pose2D p = g_odom->getPose();

  // Heading 0 deg faces +X in field frame (by convention)
  double c = std::cos(p.theta);
  double s = std::sin(p.theta);

  double tx = p.x + inches * c;
  double ty = p.y + inches * s;

  move_to_point(tx, ty, maxSpeed, timeoutMs);
}
