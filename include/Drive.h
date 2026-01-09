#pragma once

#include "main.h"
#include "Odom.h"
#include "Pid.h"

class MecanumDrive {
public:
  // Motors are provided from main.cpp (declared extern here)
  MecanumDrive(pros::Motor &fl, pros::Motor &fr, pros::Motor &bl, pros::Motor &br,
               pros::Motor &tfl, pros::Motor &tfr, pros::Motor &tbl, pros::Motor &tbr,
               Odom &odom, double maxPower = 127.0);

  void reset();
  Pose2D getPose() const;
  void setPose(const Pose2D &p);

  // Blocking go-to-pose with simple holonomic P controller.
  // Stops when within xyTol (in) and thetaTol (rad) or timeoutMs reached.
  void moveToPose(const Pose2D &target,
                  double xyKp = 4.0, double thetaKp = 2.0,
                  double xyMax = 60.0, double omegaMax = 4.0,
                  double xyTol = 1.0, double thetaTol = 5.0 * M_PI / 180.0,
                  int settleTimeMs = 150, int timeoutMs = 3000,
                  int loopDtMs = 10);

private:
  // Hardware
  pros::Motor &m_fl; // front-left
  pros::Motor &m_fr; // front-right
  pros::Motor &m_bl; // back-left
  pros::Motor &m_br; // back-right
  pros::Motor &m_tfl;
  pros::Motor &m_tfr;
  pros::Motor &m_tbl;
  pros::Motor &m_tbr;
  Odom &m_odom;

  // Params
  double m_maxPower;
  PID m_pidX{0,0,0};
  PID m_pidY{0,0,0};
  PID m_pidT{0,0,0};

  // Helpers
  void setChassisPower(double vx, double vy, double omega); // vx: forward +Y, vy: +X left
};

// Extern motors provided by main.cpp
extern pros::Motor Back_left;
extern pros::Motor Top_back_Left;
extern pros::Motor Front_left;
extern pros::Motor Top_front_Left;
extern pros::Motor front_right;
extern pros::Motor Top_front_Right;
extern pros::Motor back_right;
extern pros::Motor Top_back_Right;
