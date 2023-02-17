#include "PIDMotor.hpp"
#include <sys/types.h>

void PIDMotor::tick() {

        const double kP = 0.1;
        const double kI = 0.05;
        const double kD = 0.01;

        // Measure the current speed of the flywheel motor
        double currentSpeed = motor->motor->get_actual_velocity();
        
        // Calculate the error between the current speed and target speed
        double error = target - currentSpeed;
        
        // Add the error to the total error (for I term)
        totalError += error;
        
        // Calculate the PID output
        double output = kP * error + kI * totalError + kD * (error - prevError);
        
        // Save the current error for the next iteration
        prevError = error;
        
        // Set the motor speed to the PID output
        motor->target = output;
        motor1->target = output;
}

PIDMotor::PIDMotor(MotorH m, MotorH m1)  {
    this->motor = &m;
    this->motor1 = &m1;
} 
