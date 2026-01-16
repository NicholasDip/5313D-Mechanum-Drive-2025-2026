#include "Auton.h"
#include "motion.h"
#include "main.h"
#include "robodash/api.h"
#include "Odom.h"

// Access motors and pistons from main.cpp
extern pros::Motor intake_bottom, intake_top, intake_flex;
extern pros::adi::DigitalOut piston;
extern Odom odom;
/******************************************************************************
 *                         Motion PID Tuning 
 ******************************************************************************/
// move_to_point distance PID
double MTP_DIST_KP = 2.0;   
double MTP_DIST_KI = 0.0;
double MTP_DIST_KD = 0.5;   // Increase for more Dampening

// move_to_point heading PID
double MTP_HEAD_KP = 2.5; // Creates more agressive turning 
double MTP_HEAD_KI = 0.0;
double MTP_HEAD_KD = 0.5;   
double MTP_HEAD_MAX = 40.0 // More Turn Power      


rd::Selector selector({
    {"Red Left", red_left_auton},
    {"Red Right", red_right_auton},
    {"Blue Left", blue_left_auton},
    {"Blue Right", blue_right_auton},
    {"Skills", skills_auton},
});

void autonomous() {
    //selector.run_auton();
     odom.reset(0, 0, 0);

    move_to_point(31, 0, 0, 40, 3000);
    move_to_point(8.1 ,34.5, 150, 40, 3000);
    
}

void red_left_auton() {

}

void red_right_auton() {}
void blue_left_auton() {}
void blue_right_auton() {}
void skills_auton() {}

// ============================================================================
// JERRY.IO TEMPLATE (put in Custom Export):
//
// path: `// ${name}
// ${code}
// `
// moveToPoint: `move_to_point(${x}, ${y}, ${heading}, ${speed}, 3000);\n`
// ============================================================================
