#include "main.h"
#include "api.h"
#include "robodash/api.h"
#include "robodash/views/console.hpp"

/******************************************************************************
 *                              Motor Definitions
 ******************************************************************************/
// Drive Train
pros::Motor Back_left(-18, pros::v5::MotorGears::blue);
pros::Motor Top_back_Left(-17, pros::v5::MotorGears::green);

pros::Motor Front_left(-19, pros::v5::MotorGears::blue);
pros::Motor Top_front_Left(-20, pros::v5::MotorGears::green);

pros::Motor front_right(11, pros::v5::MotorGears::blue);
pros::Motor Top_front_Right(12, pros::v5::MotorGears::green);

pros::Motor back_right(13, pros::v5::MotorGears::blue);
pros::Motor Top_back_Right(14, pros::v5::MotorGears::green);

pros::Motor intake_bottom(9, pros::v5::MotorGears::blue);
pros::Motor intake_top(1, pros::v5::MotorGears::green);
pros::Motor intake_flex(10, pros::v5::MotorGears::green);

/******************************************************************************
 *                              Sensor Definitions
 ******************************************************************************/
pros::adi::DigitalOut piston('A'); //
bool isPistonOpen = false;

/******************************************************************************
 *                              Function Prototypes
 ******************************************************************************/
void drive_control(pros::Controller& master);
void intake_control(pros::Controller& master);

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}


// Declare console pointer at file scope
rd::Console* console = nullptr;
bool intake_flex_reversed = false;

void initialize() {
    console = new rd::Console("Motor Temps");

Back_left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
Front_left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
front_right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
back_right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
Top_front_Left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
Top_front_Right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
Top_back_Left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
Top_back_Right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
} 

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void drive_control(pros::Controller& master) {
    int y = -master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int x = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    int turn = -master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    
    Back_left.move(y + x + turn);
    Front_left.move(y - x + turn);
    back_right.move(y - x - turn);
    front_right.move(y + x - turn);
    Top_front_Left.move(y - x + turn);
    Top_back_Left.move(y + x + turn);
    Top_front_Right.move(y + x - turn);
    Top_back_Right.move(y - x - turn);
}


void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    int lastClearTime = pros::millis();
    const int clearInterval = 2000; // Clear every 2 seconds
    
    while (true) {
        // RoboDash monitoring
       console->println("Back_left: " + std::to_string(Back_left.get_temperature()));
       console->println("Front_left: " + std::to_string(Front_left.get_temperature()));
       console->println("Front_right: " + std::to_string(front_right.get_temperature()));
       console->println("Back_right: " + std::to_string(back_right.get_temperature()));
	   console->println("Top_front_Left: " + std::to_string(Top_front_Left.get_temperature()));
	   console->println("Top_front_Right: " + std::to_string(Top_front_Right.get_temperature()));
	   console->println("Top_back_Left: " + std::to_string(Top_back_Left.get_temperature()));
	   console->println("Top_back_Right: " + std::to_string(Top_back_Right.get_temperature()));
        drive_control(master);
        intake_control(master);
        
        // Pneumatic control - Toggle with L1 button
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            isPistonOpen = !isPistonOpen;
            piston.set_value(isPistonOpen);
        }
        
        if (pros::millis() - lastClearTime > clearInterval) {
            console->clear();
            lastClearTime = pros::millis();
        }
        pros::delay(20);
    }
}


/******************************************************************************
 *                           Driver Control Functions
 ******************************************************************************/
void intake_control(pros::Controller& master) {
    // Macro button to toggle intake_flex direction
    static bool last_x_state = false;
    bool current_x_state = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    
    // Toggle on button press (rising edge detection)
    if (current_x_state && !last_x_state) {
        intake_flex_reversed = !intake_flex_reversed;
    }
    last_x_state = current_x_state;
    
    // Normal intake controls
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        intake_bottom.move_velocity(200);
        intake_top.move_velocity(200);
        // Apply reversal to intake_flex if macro is active
        intake_flex.move_velocity(intake_flex_reversed ? -200 : 200);
    } 
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        intake_bottom.move_velocity(-200);
        intake_top.move_velocity(-200);
        // Apply reversal to intake_flex if macro is active
        intake_flex.move_velocity(intake_flex_reversed ? 200 : -200);
    }
    else {
        intake_bottom.move_velocity(0);
        intake_top.move_velocity(0);
        intake_flex.move_velocity(0);
    }
}