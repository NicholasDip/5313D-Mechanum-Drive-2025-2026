#include "Auton.h"
#include "motion.h"
#include "main.h"
#include "robodash/api.h"
#include "Odom.h"

// Access motors and pistons from main.cpp
extern pros::Motor intake_bottom, intake_top, intake_flex;
extern pros::adi::DigitalOut piston;
extern Odom odom;

rd::Selector selector({
  {"red_left_auton", red_left_auton},
  {"red_right_auton", red_right_auton},
  {"blue_left_auton", blue_left_auton},
  {"blue_right_auton", blue_right_auton},
  {"skills_auton", skills_auton},
});

void autonomous() {
  //selector.run_auton();
  odom.reset(-61, -17, 0);
move_to_point(-22.521, -22.462, -220, 30, 2500);
move_to_point(-46.702, -47.182, 90, 30, 2500);
move_to_point(-25.358, -47.047, 90, 30, 2500);
}

void red_left_auton() {
  // Test: forward 24", end facing 12°
  move_to_point(24, 0, 12, 60, 2500);
  move_to_point(15, 0, 0, 60, 2500);
}

void red_right_auton() {}
void blue_left_auton() {}
void blue_right_auton() {}
void skills_auton() {}





//path: `// ${name}

//${code}
//`
//moveToPoint: `move_to_point(${x}, ${y}, ${heading}, ${speed}, 2500);`