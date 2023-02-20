#include "PIDMotor.hpp"
#include "pros/rotation.hpp"
#include <sys/types.h>

void PIDMotor::tick() {
    // - PID loop -

    // Init values
    const float kP = 1.0f;
    const float kI = 0.10f;
    const float kD = 0.01f;
    const float currentSpeed = ((rot.get_velocity() / -6.0f) / 3000.0f) * 128;
    const float to = target;

    // Error calc
    float error = to - currentSpeed;
    totalError += error;
    double out = (kP * error) + (kI * totalError) + (kD * (error - prevError));

    std::cout << std::to_string(error) << std::endl;

    // Set output
    motor->target = out;
    motor1->target = out;
    prevError = error;
}

PIDMotor::PIDMotor(MotorH *m, MotorH *m1)  {
    this->motor = m;
    this->motor1 = m1;

} 
