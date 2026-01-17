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
 odom.reset(0, 0, 0);
   turn_to_angle(-24, 60, 1000);
    intake_bottom.move_velocity(600);
  intake_top.move_velocity(200);
  intake_flex.move_velocity(200);
  drive_straight(35, 35, 2000);
 
  turn_to_angle(-140, 60, 2000);   // Turn to 90 degrees heading
  drive_straight(33.5, 35, 2000);
  turn_to_angle(-188, 60, 1000);
  drive_straight(-24, 60, 1000);
  piston.set_value(true);

}

void red_left_auton() {
  // Simple drive forward test
  odom.reset(0, 0, 0);
  drive_straight(50, 60, 3000);  // Drive 50 inches forward at 60 speed
  //turn_to_angle(90, 00, 2000);   // Turn to 90 degrees heading
}

void red_right_auton() {
odom.reset(0, 0, 0);
     turn_to_angle(-26, 60, 1000);
    intake_bottom.move_velocity(600);
  intake_top.move_velocity(200);
  intake_flex.move_velocity(200);
  drive_straight(35, 35, 2000);
 
  turn_to_angle(-140, 60, 2000);   // Turn to 90 degrees heading
  drive_straight(37, 35, 2000);
  turn_to_angle(-188, 60, 1000);
  drive_straight(-36, 60, 1000);
  piston.set_value(true);

}
void blue_left_auton() {}
void blue_right_auton() {}
void skills_auton() {}





//path: `// ${name}

//${code}
//`
//moveToPoint: `move_to_point(${x}, ${y}, ${heading}, ${speed}, 2500);`

// Path

//moveToPoint(22.115, -22.471, 265, 30);
//moveToPoint(48.078, -46.162, 270, 30);
//moveToPoint(29.465, -46.984, 270, 30);


