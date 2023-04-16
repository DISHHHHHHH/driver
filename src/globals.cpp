#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor intake(5, pros::E_MOTOR_GEARSET_06,false,pros::E_MOTOR_ENCODER_COUNTS);

sylib::SpeedControllerInfo motor_speed_controller ( // read here on how to graph this for easy tuning https://sylvie.fyi/sylib/docs/dd/d2c/md_sample_code__demo.html
    //set the gain to 0 to not use it
    [](double rpm){return 5;}, // kV function (feedforward), should return x in this: voltage = x * rpm
    1, // kP
    1, // kI
    1, // kD
    1, // kH (tbh gain)
    false, // anti-windup enabled
    0, // anti-windup range
    false, // p controller bounds threshold enabled
    0, // p controller bounds cutoff enabled
    1, // kP2 for when over threshold
    0 // range to target to apply max voltage
);

auto flywheel = sylib::Motor(9,3600, true, motor_speed_controller);

// Chassis constructor
Drive chassis(
    // Left Chassis Ports (negative port will reverse it!)
    //   the first port is the sensored port (when trackers are not used!)
    {1, -2, 3}

    // Right Chassis Ports (negative port will reverse it!)
    //   the first port is the sensored port (when trackers are not used!)
    ,
    {6, -7, -8}

    // IMU Port
    ,
    4

    // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
    //    (or tracking wheel diameter)
    ,
    3.25

    // Cartridge RPM
    //   (or tick per rotation if using tracking wheels)
    ,
    600

    // External Gear Ratio (MUST BE DECIMAL)
    //    (or gear ratio of tracking wheel)
    // eg. if your drive is 84:36 where the 36t is powered, your RATIO would
    // be 2.333. eg. if your drive is 36:60 where the 60t is powered, your RATIO
    // would be 0.6.
    ,
    1.666

    // Uncomment if using tracking wheels
    /*
    // Left Tracking Wheel Ports (negative port will reverse it!)
    // ,{1, 2} // 3 wire encoder
    // ,8 // Rotation sensor

    // Right Tracking Wheel Ports (negative port will reverse it!)
    // ,{-3, -4} // 3 wire encoder
    // ,-9 // Rotation sensor
    */

    // Uncomment if tracking wheels are plugged into a 3 wire expander
    // 3 Wire Port Expander Smart Port
    // ,1
);

void fire() {

  flywheel.set_velocity_custom_controller(3600);
  pros::delay(300);
  intake.move_relative(500, 100); //idk how many ticks to move the intake  
}