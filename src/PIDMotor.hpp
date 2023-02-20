#include "main.h"
#include "pros/motors.hpp"
#include "MotorH.hpp"
#include "pros/rotation.hpp"

#ifndef GR
#define GR

extern pros::Rotation rot;
#endif

class PIDMotor {
    public:
        MotorH *motor;
        MotorH *motor1;

        PIDMotor(MotorH *m, MotorH *m1);

        double totalError = 0;
        double prevError = 0;

        int target;
        int current;

        bool isAtSpeed = false;

        void tick();
};
