#include "PIDMotor.hpp"
#include <sys/types.h>

void PIDMotor::tick() {
}

PIDMotor::PIDMotor(MotorH m, MotorH m1)  {
    this->motor = &m;
    this->motor1 = &m1;

} 
