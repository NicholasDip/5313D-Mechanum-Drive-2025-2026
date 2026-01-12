#pragma once
#include "Odom.h"

// ============================================================================
// COORDINATE SYSTEM (Path-Based / Standard VEX):
//   - Origin (0,0) at starting position
//   - +X = right, +Y = forward
//   - Heading: 0° = +Y, positive = counter-clockwise
// ============================================================================

// Call once in initialize() to link odometry
void motion_init(Odom* odom);

// Low-level drive control (no strafe for auton)
void set_drive(double forward, double turn);
void stop_drive();

// ============================================================================
// MOTION PRIMITIVES - Turn Then Drive (no strafing)
// ============================================================================

// Turn in place to face a specific angle
void turn_to_angle(double targetDeg, double maxSpeed = 80, double timeoutMs = 1500);

// Move to a point using turn-then-drive:
// 1. Turn to face the target point
// 2. Drive forward to the point
// 3. Turn to the final heading
void move_to_point(double x, double y, double heading, double maxSpeed = 80, double timeoutMs = 3000);

// Drive straight forward/backward a distance (maintains current heading)
void drive_straight(double inches, double maxSpeed = 80, double timeoutMs = 2000);
