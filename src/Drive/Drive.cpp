#include "Drive.h"
#include <algorithm>
#include <cmath>

MecanumDrive::MecanumDrive(pros::Motor &fl, pros::Motor &fr, pros::Motor &bl, pros::Motor &br,
                           pros::Motor &tfl, pros::Motor &tfr, pros::Motor &tbl, pros::Motor &tbr,
                           Odom &odom, double maxPower)
    : m_fl(fl), m_fr(fr), m_bl(bl), m_br(br),
      m_tfl(tfl), m_tfr(tfr), m_tbl(tbl), m_tbr(tbr),
      m_odom(odom), m_maxPower(maxPower) {
  reset();
}

void MecanumDrive::reset() { m_odom.reset(); }

Pose2D MecanumDrive::getPose() const { return m_odom.getPose(); }

void MecanumDrive::setPose(const Pose2D &p) { m_odom.setPose(p); }

void MecanumDrive::setChassisPower(double vx, double vy, double omega) {
  // vx: forward (+Y), vy: left (+X). omega: CCW rad/s scaled to power later
  // Convert to power mix; scale down to [-m_maxPower, m_maxPower]
  // The omega scaling is absorbed by PID output limits.
  double turn = omega;

  double fl = vx - vy + turn;
  double fr = vx + vy - turn;
  double bl = vx + vy + turn;
  double br = vx - vy - turn;

  // Normalize if any exceeds maxPower
  double maxMag = std::max({std::abs(fl), std::abs(fr), std::abs(bl), std::abs(br), 1.0});
  double scale = m_maxPower / maxMag;
  fl *= scale; fr *= scale; bl *= scale; br *= scale;

  m_fl.move(fl);
  m_tfl.move(fl);
  m_fr.move(fr);
  m_tfr.move(fr);
  m_bl.move(bl);
  m_tbl.move(bl);
  m_br.move(br);
  m_tbr.move(br);
}

void MecanumDrive::moveToPose(const Pose2D &target,
                              double xyKp, double thetaKp,
                              double xyMax, double omegaMax,
                              double xyTol, double thetaTol,
                              int settleTimeMs, int timeoutMs,
                              int loopDtMs) {
  int tStart = pros::millis();
  int settledSince = -1;
  m_pidX.reset();
  m_pidY.reset();
  m_pidT.reset();
  m_pidX.setTunings(xyKp, 0, 0);
  m_pidY.setTunings(xyKp, 0, 0);
  m_pidT.setTunings(thetaKp, 0, 0);
  m_pidX.setOutputLimits(-xyMax, xyMax);
  m_pidY.setOutputLimits(-xyMax, xyMax);
  m_pidT.setOutputLimits(-omegaMax, omegaMax);
  while (true) {
    m_odom.update();

    // Position error in field frame
    Pose2D pose = m_odom.getPose();
    double ex = target.x - pose.x;
    double ey = target.y - pose.y;

    // Rotate error into robot frame
    double c = std::cos(-pose.theta);
    double s = std::sin(-pose.theta);
    double erx = ex * c - ey * s; // right/left (+X left)
    double ery = ex * s + ey * c; // forward (+Y)

    // Heading error
    double et = target.theta - pose.theta;
    while (et > M_PI) et -= 2 * M_PI;
    while (et < -M_PI) et += 2 * M_PI;

    // Proportional control
    double dt = loopDtMs / 1000.0;
    double vx = m_pidY.step(ery, dt);
    double vy = m_pidX.step(erx, dt);
    double om = m_pidT.step(et, dt);

    setChassisPower(vx, vy, om);

    bool posOk = std::hypot(ex, ey) <= xyTol;
    bool angOk = std::abs(et) <= thetaTol;
    if (posOk && angOk) {
      if (settledSince < 0) settledSince = pros::millis();
      if (pros::millis() - settledSince >= settleTimeMs) break;
    } else {
      settledSince = -1;
    }

    if (pros::millis() - tStart > timeoutMs) break;
    pros::delay(loopDtMs);
  }

  setChassisPower(0, 0, 0);
}
