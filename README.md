# Mecha VEX Robotics Project

## Overview
This project is a PROS-based VEX V5 robot codebase using RoboDash for autonomous selection and monitoring. It is organized for clarity and modularity, with autonomous routines separated from main control logic. Set to be used during VEX Pushback 2025-2026

## Features
- Modular autonomous routines with RoboDash Selector
- Mechanum Drive Setup Code
- Real-time motor temperature monitoring
- Organized motor definitions and drive control
- Easy-to-edit autonomous code in `src/Auton.cpp`

## File Structure
- `src/main.cpp`: Main robot control, drive logic, and monitoring
- `src/Auton.cpp`: Autonomous routines and RoboDash selector setup
- `include/Auton.h`: Autonomous routine declarations
- `include/main.h`: Main header for robot code

## Autonomous Selection
Autonomous routines are defined in `Auton.cpp` and registered with RoboDash's Selector widget. The selected routine runs automatically in competition mode. one item can have is own autonmous path. 
For example {"red_left_auton", red_left_auton}, that can be coded with a path specified for red side left. Allowing versatility across the field in all scenarios 

Example:
```cpp
rd::Selector selector({
    {"red_left_auton", red_left_auton},
    {"red_right_auton", red_right_auton},
    {"blue_left_auton", blue_left_auton},
    {"blue_right_auton", blue_right_auton},
    {"skills_auton", skills_auton},
});

void autonomous() {
    selector.run_auton();
}
```

## Motor Monitoring
Motor temperatures are monitored and can be displayed using RoboDash or PROS LCD functions. Example:
```cpp
pros::lcd::set_text(1, "Front Left Temp: " + std::to_string(Front_left.get_temperature()));  /// This will appear on brain system screen 
```

## Dependencies
- [PROS](https://pros.cs.purdue.edu/)
- [RoboDash](https://robodash.readthedocs.io/en/latest/)

## License
MIT License

---
For more information, see the RoboDash documentation: https://robodash.readthedocs.io/en/latest/
