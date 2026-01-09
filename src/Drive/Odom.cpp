// ===============================
// FILE: src/Odom.cpp
// ===============================
#include "Odom.h"
#include <cmath>

Odom::Odom(pros::Rotation& verticalTracking,
           pros::Rotation& horizontalTracking,
           pros::Imu& imu,
           double trackingWheelDiameterIn,
           bool invertVertical,
           bool invertHorizontal)
  : m_vert(verticalTracking),
    m_horz(horizontalTracking),
    m_imu(imu),
    m_wheelDiam(trackingWheelDiameterIn),
    m_invertV(invertVertical),
    m_invertH(invertHorizontal) {}

void Odom::reset(double x, double y, double headingDeg) {
  // Set pose
  m_pose.x = x;
  m_pose.y = y;
  m_pose.theta = deg2rad(headingDeg);

  // Zero sensors
  m_imu.set_rotation(headingDeg);
  m_vert.reset_position();
  m_horz.reset_position();

  m_lastVDeg = 0.0;
  m_lastHDeg = 0.0;
}

void Odom::update() {
  // Rotation sensors return centidegrees
  double vDeg = m_vert.get_position() / 100.0;
  double hDeg = m_horz.get_position() / 100.0;

  double dv = vDeg - m_lastVDeg;
  double dh = hDeg - m_lastHDeg;

  m_lastVDeg = vDeg;
  m_lastHDeg = hDeg;

  // Convert delta degrees -> inches
  double dV = (dv / 360.0) * (M_PI * m_wheelDiam);
  double dH = (dh / 360.0) * (M_PI * m_wheelDiam);

  if (m_invertV) dV = -dV;
  if (m_invertH) dH = -dH;

  // Heading from IMU (deg -> rad)
  double newTheta = deg2rad(m_imu.get_rotation());
  double dTheta = wrapRad(newTheta - m_pose.theta);

  // Use mid-step heading for better accuracy during turns
  double thetaMid = wrapRad(m_pose.theta + dTheta * 0.5);
  m_pose.theta = wrapRad(m_pose.theta + dTheta);

  double c = std::cos(thetaMid);
  double s = std::sin(thetaMid);

  // Robot-frame deltas:
  //  dV = forward, dH = right (by our convention)
  // Field update:
  m_pose.x += dV * c - dH * s;
  m_pose.y += dV * s + dH * c;
}
