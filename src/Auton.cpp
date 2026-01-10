#include "Auton.h"
#include "motion.h"
#include "main.h"
#include "robodash/api.h"

// Access motors and pistons from main.cpp
extern pros::Motor intake_bottom, intake_top, intake_flex;
extern pros::adi::DigitalOut piston;

rd::Selector selector({
  {"red_left_auton", red_left_auton},
  {"red_right_auton", red_right_auton},
  {"blue_left_auton", blue_left_auton},
  {"blue_right_auton", blue_right_auton},
  {"skills_auton", skills_auton},
});

void autonomous() {
  //selector.run_auton();
  
intake_bottom.move_velocity(600);  
intake_top.move_velocity(200);     
intake_flex.move_velocity(200);
 turn_to_angle(32, 90, 1500);
 drive_straight(40, 60, 2500);
}

void red_left_auton() {
  // Test: forward 24"
  move_to_point(24, 0, 0, 2500);
  turn_to_angle(12, 60, 1500);
  move_to_point(15, 0, 0, 2500);
}

void red_right_auton() {}
void blue_left_auton() {}
void blue_right_auton() {}
void skills_auton() {}
