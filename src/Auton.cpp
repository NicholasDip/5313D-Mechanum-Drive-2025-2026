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
double MTP_DIST_KP = 3.8; //Adjust this first till target reach
double MTP_DIST_KI = 0.0;
double MTP_DIST_KD = 0.0;   // Increase for more Dampening / Overshute

// move_to_point heading PID
double MTP_HEAD_KP = 1.1; // Higher KP since KD will dampen
double MTP_HEAD_KI = 0.0;
double MTP_HEAD_KD = 0.0;  // Damping to prevent overshoot  
double MTP_HEAD_MAX = 60.0; // More Turn Power      
double MTP_HEAD_MIN = 20.0; // Less Turn Power

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

    move_to_point(24, 0, 0, 30, 3500);
    move_to_point(8.1 , 35, 165, 35, 3000);
    
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
