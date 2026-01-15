#pragma once
#include "Odom.h"

// ============================================================================
// COORDINATE SYSTEM (Path-Based):
//   - Origin (0,0) at starting position
//   - +X = forward (purple axis)
//   - +Y = right (red axis)
//   - Heading: 0° = facing +X, 90° = facing +Y, clockwise positive
// ============================================================================

// Call once in initialize() to link odometry
void motion_init(Odom* odom);

// Low-level drive control with strafe
void set_drive(double forward, double strafe, double turn);
void stop_drive();

// ============================================================================
// MOTION PRIMITIVES - Holonomic (with strafing)
// ============================================================================

// Turn in place to face a specific angle
void turn_to_angle(double targetDeg, double maxSpeed = 80, double timeoutMs = 1500);

// Drive straight forward/backward (maintains current heading)
void drive_straight(double inches, double maxSpeed = 80, double timeoutMs = 2000);

// Strafe left/right (maintains current heading)
// Positive = right, Negative = left
void strafe(double inches, double maxSpeed = 80, double timeoutMs = 2000);

// Move to a point using holonomic drive (can strafe and drive simultaneously)
// Robot will take the most efficient path and end at the specified heading
void move_to_point(double x, double y, double heading, double maxSpeed = 80, double timeoutMs = 3000);
