#include "main.h"
#include "pros/motors.hpp"
#include "MotorH.hpp"
#include "globals.hpp"

class PIDMotor {
    public:
        MotorH *motor;
        MotorH *motor1;

        PIDMotor(MotorH m, MotorH m1);

        double totalError = 0;
        double prevError = 0;

        int target;
        int current;
        
        void tick();
};
