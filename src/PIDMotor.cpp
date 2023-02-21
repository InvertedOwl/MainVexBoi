#include "PIDMotor.hpp"
#include "pros/rotation.hpp"
#include <sys/types.h>

void PIDMotor::tick() {
    // - P loop -

    // Init values
    //const float kP = .07f;
    const float kP = .08f;
    const float currentSpeed = ((rot.get_velocity() / -6.0f) / 3116.66f) * 127; // DPS to -127 127 for error handling
    // Error calc
    float error = target - currentSpeed;

    if (std::abs(error) < kP) {
        isAtSpeed = true;
    } else {
        isAtSpeed = false;
    }
    // Set output
    motor->target = target + (error * kP);
    motor1->target = target + (error * kP);
    // prevError = error;
}

PIDMotor::PIDMotor(MotorH *m, MotorH *m1)  {
    this->motor = m;
    this->motor1 = m1;

} 
