#pragma once
#include "robodash/api.h"

// auton entry
#ifdef __cplusplus
extern "C" {
#endif

void autonomous();

#ifdef __cplusplus
}
#endif

// routines
void red_left_auton();
void red_right_auton();
void blue_left_auton();
void blue_right_auton();
void skills_auton();

// selector (defined in Auton.cpp)
extern rd::Selector selector;
