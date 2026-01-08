#include "Auton.h"
#include "main.h"
#include "robodash/api.h"
#include <cmath>
#include <vector>

rd::Selector selector({
    {"red_left_auton", red_left_auton},
    {"red_right_auton", red_right_auton},
    {"blue_left_auton", blue_left_auton},
    {"blue_right_auton", blue_right_auton},
    {"skills_auton", skills_auton},
});

/******************************************************************************
 *                           Odometry Variables
 ******************************************************************************/
double robot_x = 0.0;
double robot_y = 0.0;
double robot_heading = 0.0;  // in degrees
double last_left_pos = 0.0;
double last_right_pos = 0.0;

// Odometry constants (defined in main.cpp)
extern const double WHEEL_DIAMETER;
extern const double TRACK_WIDTH;
extern const double TICKS_PER_REV;

// Motor and sensor references (defined in main.cpp)
extern pros::Motor Back_left, Front_left, back_right, front_right;
extern pros::Motor Top_back_Left, Top_front_Left, Top_back_Right, Top_front_Right;
extern pros::Imu imu;
extern pros::Rotation left_tracking, right_tracking;

/******************************************************************************
 *                           PID Controller
 ******************************************************************************/
PID::PID(double p, double i, double d, double int_limit) 
    : kP(p), kI(i), kD(d), integral(0), previous_error(0), integral_limit(int_limit) {}

double PID::calculate(double error) {
    integral += error;
    
    // Anti-windup
    if (integral > integral_limit) integral = integral_limit;
    if (integral < -integral_limit) integral = -integral_limit;
    
    double derivative = error - previous_error;
    previous_error = error;
    
    return (kP * error) + (kI * integral) + (kD * derivative);
}

void PID::reset() {
    integral = 0;
    previous_error = 0;
}

/******************************************************************************
 *                           Odometry Functions
 ******************************************************************************/
void update_odometry() {
    // Get current encoder positions (in degrees)
    double left_pos = left_tracking.get_position() / 100.0;  // Convert centidegrees to degrees
    double right_pos = right_tracking.get_position() / 100.0;
    
    // Calculate change in position
    double delta_left = (left_pos - last_left_pos) * (M_PI / 180.0) * (WHEEL_DIAMETER / 2.0);
    double delta_right = (right_pos - last_right_pos) * (M_PI / 180.0) * (WHEEL_DIAMETER / 2.0);
    
    last_left_pos = left_pos;
    last_right_pos = right_pos;
    
    // Get heading from IMU
    robot_heading = imu.get_rotation();
    
    // Calculate change in position
    double delta_center = (delta_left + delta_right) / 2.0;
    
    // Update global position
    double heading_rad = robot_heading * (M_PI / 180.0);
    robot_x += delta_center * cos(heading_rad);
    robot_y += delta_center * sin(heading_rad);
}

void reset_odometry(double x, double y, double heading) {
    robot_x = x;
    robot_y = y;
    robot_heading = heading;
    
    imu.set_rotation(heading);
    left_tracking.reset_position();
    right_tracking.reset_position();
    
    last_left_pos = 0.0;
    last_right_pos = 0.0;
}

void get_position(double& x, double& y, double& heading) {
    x = robot_x;
    y = robot_y;
    heading = robot_heading;
}

/******************************************************************************
 *                           Drive Functions
 ******************************************************************************/
void set_drive(int left, int right) {
    Back_left.move(left);
    Front_left.move(left);
    Top_back_Left.move(left);
    Top_front_Left.move(left);
    
    back_right.move(right);
    front_right.move(right);
    Top_back_Right.move(right);
    Top_front_Right.move(right);
}

void turn_to_angle(double target_angle, double max_speed, double timeout) {
    PID turn_pid(2.5, 0.01, 8.0);  // Tune these values
    
    uint32_t start_time = pros::millis();
    
    while (pros::millis() - start_time < timeout) {
        update_odometry();
        
        // Calculate error (-180 to 180 range)
        double error = target_angle - robot_heading;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        
        // Exit if close enough
        if (fabs(error) < 1.0) {
            break;
        }
        
        double turn_power = turn_pid.calculate(error);
        
        // Limit speed
        if (turn_power > max_speed) turn_power = max_speed;
        if (turn_power < -max_speed) turn_power = -max_speed;
        
        set_drive(turn_power, -turn_power);
        pros::delay(10);
    }
    
    set_drive(0, 0);
}

void move_to_point(double target_x, double target_y, double max_speed, double timeout) {
    PID distance_pid(8.0, 0.0, 2.0);  // Tune these values
    PID heading_pid(2.0, 0.0, 1.0);
    
    uint32_t start_time = pros::millis();
    
    while (pros::millis() - start_time < timeout) {
        update_odometry();
        
        // Calculate distance and angle to target
        double dx = target_x - robot_x;
        double dy = target_y - robot_y;
        double distance = sqrt(dx * dx + dy * dy);
        double target_angle = atan2(dy, dx) * (180.0 / M_PI);
        
        // Exit if close enough
        if (distance < 0.5) {
            break;
        }
        
        // Calculate heading error
        double heading_error = target_angle - robot_heading;
        while (heading_error > 180) heading_error -= 360;
        while (heading_error < -180) heading_error += 360;
        
        // Calculate motor powers
        double forward_power = distance_pid.calculate(distance);
        double turn_power = heading_pid.calculate(heading_error);
        
        // Limit forward speed
        if (forward_power > max_speed) forward_power = max_speed;
        
        // Calculate left and right motor powers
        double left_power = forward_power + turn_power;
        double right_power = forward_power - turn_power;
        
        set_drive(left_power, right_power);
        pros::delay(10);
    }
    
    set_drive(0, 0);
}

void drive_distance(double inches, double max_speed, double timeout) {
    double current_x, current_y, current_heading;
    get_position(current_x, current_y, current_heading);
    
    // Calculate target position
    double heading_rad = current_heading * (M_PI / 180.0);
    double target_x = current_x + inches * cos(heading_rad);
    double target_y = current_y + inches * sin(heading_rad);
    
    move_to_point(target_x, target_y, max_speed, timeout);
}

void follow_path(const std::vector<Waypoint>& points, double timeout_per_point) {
    for (const auto& wp : points) {
        move_to_point(wp.x, wp.y, wp.speed, timeout_per_point);
        pros::delay(50);
    }
}

std::vector<Waypoint> field_to_local(const std::vector<Waypoint>& field_points,
                                     double start_field_x,
                                     double start_field_y,
                                     double start_field_heading_deg) {
    std::vector<Waypoint> local;
    local.reserve(field_points.size());
    const double th = start_field_heading_deg * (M_PI / 180.0);
    const double c = cos(th);
    const double s = sin(th);
    for (const auto& fp : field_points) {
        const double dx = fp.x - start_field_x;
        const double dy = fp.y - start_field_y;
        // Robot-local: X forward, Y left (standard robotics body frame)
        const double xr =  c * dx + s * dy;
        const double yl = -s * dx + c * dy;
        local.push_back(Waypoint{xr, yl, fp.speed});
    }
    return local;
}

void follow_field_path(const std::vector<Waypoint>& field_points,
                       double start_field_x,
                       double start_field_y,
                       double start_field_heading_deg,
                       double timeout_per_point) {
    auto local = field_to_local(field_points, start_field_x, start_field_y, start_field_heading_deg);
    follow_path(local, timeout_per_point);
}


void autonomous() { 
    selector.run_auton();
}

// Define autonomous routines
void red_left_auton(


    
) {
    // Reset starting position (x=0, y=0, heading=0)
    reset_odometry(0, 0, 0);
    
    // Example: Move to point (24, 24) - 24 inches forward and 24 inches right
    move_to_point(24, 24, 80);
    pros::delay(500);
    
    // Turn to face 90 degrees
    turn_to_angle(90);
    pros::delay(500);
    
    // Move to another point
    move_to_point(24, 48, 80);
    pros::delay(500);
    
    // Drive backward 12 inches
    drive_distance(-12, 60);
}

void red_right_auton() {
    // Red right autonomous code
}

void blue_left_auton() {
    // Blue left autonomous code
}

void blue_right_auton() {
    // Blue right autonomous code
}

void skills_auton() {
    // Skills autonomous code
}


