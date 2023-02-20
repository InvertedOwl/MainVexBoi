#include "PIDMotor.hpp"
#include "pros/rotation.hpp"
#include <sys/types.h>

void PIDMotor::tick() {
    // - P loop -

    // Init values
    const float kP = 1.0f;
    const float currentSpeed = ((rot.get_velocity() / -6.0f) / 3000.0f) * 128; // DPS to -127 127 for error handling

    // Error calc
    float error = target - currentSpeed;

    if (std::abs(error) < 1) {
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
