#pragma once

#include "main.h"
#include "robodash/api.h"
#include <vector>

/**
 * @brief PID Controller Class
 */
class PID {
private:
    double kP, kI, kD;
    double integral;
    double previous_error;
    double integral_limit;
    
public:
    PID(double p, double i, double d, double int_limit = 50);
    double calculate(double error);
    void reset();
};

/**
 * @brief Odometry Functions
 */
void update_odometry();
void reset_odometry(double x = 0.0, double y = 0.0, double heading = 0.0);
void get_position(double& x, double& y, double& heading);

/**
 * @brief Movement Functions
 */
void set_drive(int left, int right);
void turn_to_angle(double target_angle, double max_speed = 80, double timeout = 2000);
void move_to_point(double target_x, double target_y, double max_speed = 100, double timeout = 3000);
void drive_distance(double inches, double max_speed = 100, double timeout = 2000);

/**
 * @brief Waypoint path helpers
 */
struct Waypoint {
    double x;
    double y;
    double speed; // 0-127 recommended, we cap internally
};

void follow_path(const std::vector<Waypoint>& points, double timeout_per_point = 3000);

// Convert field-frame points (VGSP, inches; origin=field center) to robot-local
// Robot-local frame: X forward, Y left (positive left). If you prefer Y=right, flip sign of Y.
std::vector<Waypoint> field_to_local(const std::vector<Waypoint>& field_points,
                                     double start_field_x,
                                     double start_field_y,
                                     double start_field_heading_deg);

// Convenience: follow a path defined in field frame.
void follow_field_path(const std::vector<Waypoint>& field_points,
                       double start_field_x,
                       double start_field_y,
                       double start_field_heading_deg,
                       double timeout_per_point = 3000);

/**
 * @brief Autonomous Routines
 */



 
void red_left_auton();
void red_right_auton();
void blue_left_auton();
void blue_right_auton();
void skills_auton();

// Robodash selector (declared in auton.cpp)
extern rd::Selector selector;
