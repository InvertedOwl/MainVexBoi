#include "PIDMotor.hpp"
#include <sys/types.h>
#include "globals.hpp"

void PIDMotor::tick() {
        const float calculatedRPM = flywheelSpeed.get_velocity() / 6.0f; // Degrees per second to RPM
        const float calculatedTarget = (target/127.0f) * 3000; // RPM

        const double kP = 0.1;
        const double kI = 0.05;
        const double kD = 0.01;

        // Measure the current speed of the flywheel motor
        double currentSpeed = calculatedRPM;
        
        // Calculate the error between the current speed and target speed
        double error = calculatedTarget - currentSpeed;
        
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
