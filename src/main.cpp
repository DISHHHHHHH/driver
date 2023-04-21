#include "main.h"
#include "EZ-Template/util.hpp"
#include "autons.hpp"
#include "globals.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "sylib/system.hpp"

/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {

  pros::Task intake_task(intake_task_fn);

  // Print our branding over your terminal :D
  ez::print_ez_template();

  pros::delay(
      500); // Stop the user from doing anything while legacy ports configure.

  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(
      true); // Enables modifying the controller curve with buttons on the
             // joysticks
  chassis.set_active_brake(0); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(
      0, 0); // Defaults for curve. If using tank, only the first parameter is
             // used. (Comment this line out if you have an SD card!)
  garage_constants(); // Set the drive to your own constants from autons.cpp!
  exit_condition_defaults(); // Set the exit conditions to your own constants
                             // from autons.cpp!

  // These are already defaulted to these buttons, but you can change the
  // left/right curve buttons here! chassis.set_left_curve_buttons
  // (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If
  // using tank, only the left side is used.
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,
  // pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons(
      {
        Auton("right", rightSide),
        Auton("left 11 disc", left11Disc),
        Auton("left", left8Disc),
        Auton("right side push", rightPushRoller),
        Auton("Teamwork Match, Left Side\n\nFull Routine", matchLeftFull),
        Auton("Teamwork Match, Push Disks In", pushAuton),
        Auton("Teamwork Match NO AUTON", matchNoAuton),
        Auton("Trust Alliance", trustAlliance),
        Auton("Test Drive\n\nDrive forward and come back.", drive_example),
        Auton("turn test\n\ntest turn", turn_test)});

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  sylib::initialize();

  // Initialize tasks
  pros::Task light_task(light_task_fn);
  pros::Task cata_task(cata_task_fn);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  whatTheLightDoin = "urple_bright";
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

void autonomous() {

  whatTheLightDoin = "urple_bright";

  chassis.reset_pid_targets();               // Resets PID targets to 0
  chassis.reset_gyro();                      // Reset gyro position to 0
  chassis.reset_drive_sensor();              // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps
                                             // autonomous consistency.


  ez::as::auton_selector
      .call_selected_auton(); // Calls selected auton from autonomous selector.
}

void opcontrol() {

  whatTheLightDoin = "urple_dim";
  

  chassis.set_drive_brake(MOTOR_BRAKE_COAST);

  pros::ADIDigitalOut piston('B');
  piston.set_value(false);

  useAltLimitSwitch = false;

  while (true) {

    chassis.arcade_standard(ez::SPLIT);

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
      fire();
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      pisstake.set_value(1);
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      pisstake.set_value(0);
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) &&
        master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      piston.set_value(true);
    }

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!
                                       // Keep this ez::util::DELAY_TIME
  }
}
