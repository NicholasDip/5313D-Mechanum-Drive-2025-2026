#include "Auton.h"
#include "main.h"
#include "robodash/api.h"
#include <cmath>
#include <vector>

rd::Selector selector({
    {"Red Left", red_left_auton},
    {"Red Right", red_right_auton},
    {"Blue Left", blue_left_auton},
    {"Blue Right", blue_right_auton},
    {"Skills", skills_auton},
});

// ============================================================================
// AUTONOMOUS ENTRY POINT
// ============================================================================

void autonomous() {
    //selector.run_auton();
 // Starting position: (0, 0) facing 90°
    odom.reset(0, 0, 90);

    move_to_point(-32.009, -21.507, 90, 30, 3000);
    move_to_point(-48.322, -47.299, 210, 30, 3000);
    //move_to_point(-36.178, -47.742, 90, 30, 3000);
    //move_to_point(-58.758, -47.932, 90, 30, 3000);
    //move_to_point(-29.41, -48.121, 90, 30, 3000);

}

// ============================================================================
// AUTONOMOUS ROUTINES
// ============================================================================

void red_left_auton() {
    // Jerry.io Path Test - Z pattern
    // Starting position: (0, 0) facing 90°
    odom.reset(0, 0, 90);

    move_to_point(-32.009, -21.507, 90, 30, 3000);
    move_to_point(-48.322, -47.299, 210, 30, 3000);
   // move_to_point(-36.178, -47.742, 90, 30, 3000);
    //move_to_point(-58.758, -47.932, 90, 30, 3000);
    //move_to_point(-29.41, -48.121, 90, 30, 3000);
}

void red_right_auton() {
    odom.reset(0, 0, 0);

    // Paste Jerry.io exported waypoints here
}

void blue_left_auton() {
    odom.reset(0, 0, 0);
}

void blue_right_auton() {
    odom.reset(0, 0, 0);
}

void skills_auton() {
    odom.reset(0, 0, 0);

    // Example using Waypoint array (alternative to individual calls)
    // Waypoint path[] = {
    //     WAYPOINT(24, 0, 0, 70),
    //     WAYPOINT(24, 24, 90, 70),
    //     WAYPOINT(0, 0, 0, 70),
    // };
    // follow_path(path, 3);
}

// ============================================================================
// JERRY.IO EXPORT TEMPLATE CONFIGURATION
// ============================================================================
//
// Go to path.jerryio.com -> Settings -> Custom Export
//
// Use these templates:
//
// PATH TEMPLATE:
// ```
// // ${name}
// // Starting position: (${startX}, ${startY}) heading ${startHeading}°
// odom.reset(${startX}, ${startY}, ${startHeading});
// ${code}
// ```
//
// POINT TEMPLATE (moveToPoint):
// ```
// move_to_point(${x}, ${y}, ${heading}, ${speed}, 3000);
// ```
//
// COORDINATE SETTINGS:
// - Unit: Inches
// - Field: VEX (144 x 144)
// - Origin: Center (0,0)
// - Heading: 0° = Up (+Y), positive = CCW
//
// ============================================================================
