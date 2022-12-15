#pragma once 

#include "pros/motors.hpp"
class MotorH {
    public:
        pros::Motor *motor;

        int target;
        int slew;
        int current;

        bool breaking;

        void breakk();

        void unbreakk();

        MotorH(int port, int slew);

        void tick();

        int getMotorPos();
};

