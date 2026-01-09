#include "Auton.h"
#include "motion.h"
#include "main.h"
#include "robodash/api.h"

rd::Selector selector({
  {"red_left_auton", red_left_auton},
  {"red_right_auton", red_right_auton},
  {"blue_left_auton", blue_left_auton},
  {"blue_right_auton", blue_right_auton},
  {"skills_auton", skills_auton},
});

void autonomous() {
  //selector.run_auton();
  move_to_point(24, 0, 60, 2500);
  
}

void red_left_auton() {
  // Test: forward 24"
  move_to_point(24, 0, 60, 2500);
}

void red_right_auton() {}
void blue_left_auton() {}
void blue_right_auton() {}
void skills_auton() {}
