#pragma once
#include "Odom.h"

// ============================================================================
// COORDINATE SYSTEM (Standard VEX):
//   - Origin (0,0) at field center
//   - +X = right (when viewed from above, driver perspective)
//   - +Y = forward (away from driver)
//   - Heading: 0° = facing +Y, positive = counter-clockwise
// ============================================================================

// Waypoint for path following (matches Jerry.io export)
struct Waypoint {
    double x;       // inches, field X coordinate
    double y;       // inches, field Y coordinate
    double heading; // degrees, final heading at this point
    double speed;   // max speed (0-127)
};

// Call once in initialize() to link odometry
void motion_init(Odom* odom);

// Low-level drive control (forward/strafe/turn all in -127 to 127 range)
void set_drive(double forward, double turn);
void stop_drive();

// ============================================================================
// MOTION PRIMITIVES - Turn Then Drive (no strafing)
// ============================================================================

// Turn in place to face a specific angle
// - Blocks until within 2° of target or timeout
// - Uses settling to ensure robot has stopped
void turn_to_angle(double targetDeg, double maxSpeed = 80, double timeoutMs = 1500);

// Move to a point using turn-then-drive:
// 1. Turn to face the target point
// 2. Drive forward to the point
// 3. Turn to the final heading
void move_to_point(double x, double y, double heading, double maxSpeed = 80, double timeoutMs = 3000);

// Drive straight forward/backward a distance (maintains current heading)
void drive_straight(double inches, double maxSpeed = 80, double timeoutMs = 2000);

// ============================================================================
// PATH FOLLOWING
// ============================================================================

// Follow a sequence of waypoints
// - Calls move_to_point for each waypoint
// - Use for Jerry.io exported paths
void follow_path(const Waypoint* path, int numPoints);

// Convenience macro for creating waypoint arrays
#define WAYPOINT(x, y, heading, speed) {x, y, heading, speed}
