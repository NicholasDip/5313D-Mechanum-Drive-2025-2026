#pragma once
#include "Odom.h"

// Call once in initialize()
void motion_init(Odom* odom);

// forward: +forward inches-direction, strafe: +right, turn: +CCW
void set_drive_mecanum(double forward, double strafe, double turn);

// Motion primitives
void turn_to_angle(double targetDeg, double max_speed = 90, double timeoutMs = 1500);
void move_to_point(double targetX, double targetY, double max_speed = 90, double timeoutMs = 2500);
void drive_distance(double inches, double max_speed = 90, double timeoutMs = 2500);
