#include "MotorH.hpp"
#include "main.h"
#include "pros/motors.h"
#include <string>

MotorH::MotorH(int port, int accelSlew)  {
    this->motor = new pros::Motor(port);

    this->target = 0;
    this->current = 0;
    this->accelSlew = accelSlew;
    this->decelSlew = accelSlew;
    motor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    motor->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
}

MotorH::MotorH(int port, int accelSlew, int decelSlew)  {

    this->motor = new pros::Motor(port);

    this->target = 0;
    this->current = 0;
    this->accelSlew = accelSlew;
    this->decelSlew = decelSlew;
    motor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    motor->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
} 

void MotorH::breakk() {
    // motor->brake();
    // breaking = true;

    // current = target;
}

void MotorH::unbreakk() {
    breaking = false;
}

void MotorH::tick() {

    // Working slew calc
    if (this->current < this->target) {
        this->current += (std::min(this->target-this->current, this->accelSlew));
    }
    if (this->current > this->target) {
        this->current += (std::max(this->target-this->current, -this->decelSlew));
    }


    // if (!breaking) {
    motor->move(this->current);
               
    // } 
    std::cout << std::to_string(current) << std::endl;

}

int MotorH::getMotorPos() {
    return (this->motor)->get_position();
}