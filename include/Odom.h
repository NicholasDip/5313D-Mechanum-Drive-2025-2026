// ===============================
// FILE: include/Odom.h
// (Tracking-wheel odom + IMU heading)
// Coordinate convention used here:
//   - Field X: forward when heading = 0°
//   - Field Y: right when heading = 0°
//   - Heading: degrees from IMU, CCW positive
// ===============================
#pragma once
#include "api.h"

struct Pose2D {
  double x;      // inches
  double y;      // inches
  double theta;  // radians
};

class Odom {
public:
  Odom(pros::Rotation& verticalTracking,
       pros::Rotation& horizontalTracking,
       pros::Imu& imu,
       double trackingWheelDiameterIn,
       bool invertVertical = false,
       bool invertHorizontal = false);

  void reset(double x = 0.0, double y = 0.0, double headingDeg = 0.0);
  void update();

  Pose2D getPose() const { return m_pose; }
  void setPose(const Pose2D& p) { m_pose = p; }

  double getX() const { return m_pose.x; }
  double getY() const { return m_pose.y; }
  double getHeadingRad() const { return m_pose.theta; }
  double getHeadingDeg() const { return m_pose.theta * 180.0 / M_PI; }

private:
  pros::Rotation& m_vert;
  pros::Rotation& m_horz;
  pros::Imu& m_imu;

  Pose2D m_pose{0, 0, 0};

  double m_wheelDiam;
  bool m_invertV;
  bool m_invertH;

  double m_lastVDeg = 0.0;
  double m_lastHDeg = 0.0;

  static inline double deg2rad(double d) { return d * M_PI / 180.0; }
  static inline double wrapRad(double r) {
    while (r > M_PI) r -= 2 * M_PI;
    while (r < -M_PI) r += 2 * M_PI;
    return r;
  }
};
