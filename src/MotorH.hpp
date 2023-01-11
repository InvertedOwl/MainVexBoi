#pragma once 

#include "pros/motors.hpp"
class MotorH {
    public:
        pros::Motor *motor;

        int target;
        int accelSlew;
        int decelSlew;
        int current;

        bool breaking;

        void breakk();

        void unbreakk();

        MotorH(int port, int slew);
        MotorH(int port, int accelSlew, int decelSlew);


        void tick();

        int getMotorPos();
};

