#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"

pros::Motor flywheel(10, pros::E_MOTOR_GEARSET_06,true,pros::E_MOTOR_ENCODER_COUNTS);

void flywheelPID(double target) {
  // Constants
  double kP = 0.2;
  double kV = .0354; 
  double threshold = 150;

  double error = 0;
  double prevError = 0;

  double output = 0;

  while (true) {

    // Proportional
    error = target - flywheel.get_actual_velocity()*6;

    // Set speed of flywheel
    if (error > threshold){
      output = 127;
    }
    else if (error < -threshold){
      output = 0;
    }
    else{
      output = (kV * target) + (kP * error) ;
    }

    // Sets the speed of the flywheel
    
    if(output > 127){
      output = 127;
    }
    else if(output < 0){
      output = 0; 
    }

    flywheel.move(output);

    prevError = error;
    pros::delay(10);

  }
}

void set_flywheel_speed(int speed) {
  static std::unique_ptr<pros::Task> pidTask {};
  if (pidTask != nullptr) { pidTask->remove(); }
  pidTask = (speed == -1) ? nullptr : std::make_unique<pros::Task>([=]{ flywheelPID(speed); });
}
