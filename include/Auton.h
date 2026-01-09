#pragma once
#include "robodash/api.h"

// auton entry
void autonomous();

// routines
void red_left_auton();
void red_right_auton();
void blue_left_auton();
void blue_right_auton();
void skills_auton();

// selector (defined in Auton.cpp)
extern rd::Selector selector;
