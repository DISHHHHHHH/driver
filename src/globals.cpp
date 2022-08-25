#include "main.h"
#include "pros/motors.hpp"
okapi::MotorGroup spinnyman({-9, 10});
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor front_intake(6);
pros::Motor gate(1, pros::E_MOTOR_GEARSET_18, false,
                 pros::E_MOTOR_ENCODER_DEGREES);
bool flywheel_state{false};
bool intake_state{true};
bool gate_state{true};

std::shared_ptr<ChassisController> chassis =
    ChassisControllerBuilder()
        .withMotors(1, -2) // left motor is 1, right motor is 2 (reversed)
        .withGains({0.001, 0, 0.0001}, // distance controller gains
                   {0.001, 0, 0.0001}, // turn controller gains
                   {0.001, 0, 0.0001}
                   // angle controller gains (helps drive straight)
                   )
        .withSensors(
            ADIEncoder{'A', 'B'}, // left encoder in ADI ports A & B
            ADIEncoder{'C', 'D',
                       true},    // right encoder in ADI ports C & D (reversed)
            ADIEncoder{'E', 'F'} // middle encoder in ADI ports E & F
            )
        // green gearset, tracking wheel diameter (2.75 in), track (7 in), and
        // TPR (360) 1 inch middle encoder distance, and 2.75 inch middle wheel
        // diameter
        .withDimensions(AbstractMotor::gearset::green,
                        {{2.75_in, 7_in, 1_in, 2.75_in}, quadEncoderTPR})
        .withOdometry()   // use the same scales as the chassis (above)
        .buildOdometry(); // build an odometry chassis

// odom

int degrees;
double posX, posY;
bool odom_state{false};