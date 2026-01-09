#pragma once
#include "api.h"
#include "Odom.h"

void motion_init(Odom* odom);

// Drive: forward(+), strafe right(+), turn CCW(+)
void set_drive_mecanum(double forward, double strafe, double turn);

// Motion
void turn_to_angle(double targetDeg, double maxSpeed = 90, double timeoutMs = 1500);
void move_to_point(double targetX, double targetY, double maxSpeed = 90, double timeoutMs = 2500);
void drive_distance(double inches, double maxSpeed = 90, double timeoutMs = 2500);
