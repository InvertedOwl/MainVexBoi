#include "main.h"
#include "globals.hpp"
#include "pros/adi.h"
#include "pros/ext_adi.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <exception>
#include <string>

int drift = 0;
int last = 0;

void stop() {
    r1.target = -0;
    r2.target = -0;
    r3.target = -0;
    l1.target = 0;
    l2.target = 0;
    l3.target = 0;
}


void forwardDist(int mmDist) {
    if (!mmDist) {
        mmDist = 20;
    }

    r1.target = -127;
    r2.target = -127;
    r3.target = -127;
    l1.target = 127;
    l2.target = 127;
    l3.target = 127;

    int initalDegrees = r1.getMotorPos();
    int target = ((mmDist / 319.28) * -360) + initalDegrees;

    int currentDegrees = r1.getMotorPos();

    while (currentDegrees > target) {
        
        try {
            currentDegrees = r1.getMotorPos();
        } catch (std::exception e) {
            std::cout << e.what() << std::endl;
        }

    }
    
    stop();

    r1.breakk();
    r2.breakk();
    r3.breakk();
    l1.breakk();
    l2.breakk();
    l3.breakk();

    pros::delay(100);

    r1.unbreakk();
    r2.unbreakk();
    r3.unbreakk();
    l1.unbreakk();
    l2.unbreakk();
    l3.unbreakk();
}

void forwardDist(void* mmDi) {

    int mmDist = (int)mmDi;

    r1.target = -127;
    r2.target = -127;
    r3.target = -127;
    l1.target = 127;
    l2.target = 127;
    l3.target = 127;

    int initalDegrees = r1.getMotorPos();
    int target = ((mmDist / 319.28) * -360) + initalDegrees;

    int currentDegrees = r1.getMotorPos();

    while (currentDegrees > target) {
        
        try {
            currentDegrees = r1.getMotorPos();
        } catch (std::exception e) {
            std::cout << e.what() << std::endl;
        }

    }
    
    stop();

    r1.breakk();
    r2.breakk();
    r3.breakk();
    l1.breakk();
    l2.breakk();
    l3.breakk();

    pros::delay(100);

    r1.unbreakk();
    r2.unbreakk();
    r3.unbreakk();
    l1.unbreakk();
    l2.unbreakk();
    l3.unbreakk();
}

void backDist(int mmDist) {

    r1.target = 127;
    r2.target = 127;
    r3.target = 127;
    l1.target = -127;
    l2.target = -127;
    l3.target = -127;

    int initalDegrees = r1.getMotorPos();
    int target = ((mmDist / 319.28) * 360) + initalDegrees;

    int currentDegrees = r1.getMotorPos();

    while (currentDegrees < target) {
        int currentDegrees = r1.getMotorPos();

        if (currentDegrees > target) {
            break;
        }
    }

    stop();

    r1.breakk();
    r2.breakk();
    r3.breakk();
    l1.breakk();
    l2.breakk();
    l3.breakk();

    pros::delay(100);

    r1.unbreakk();
    r2.unbreakk();
    r3.unbreakk();
    l1.unbreakk();
    l2.unbreakk();
    l3.unbreakk();

}

// TODO: Foot/inches

void rotateClockwise(int degrees) {

    int out = 127;
    if (degrees < 0) {
        out *= -1;
    }

    r1.target = out;
    r2.target = out;
    r3.target = out;
    l1.target = out;
    l2.target = out;
    l3.target = out;  

    pros::delay(std::abs(degrees) * 3);
    out = 0;

    r1.target = out;
    r2.target = out;
    r3.target = out;
    l1.target = out;
    l2.target = out;
    l3.target = out;  
}

void shoot(int disc, int percPower) {
    f1.target = (percPower * 0.01f) * 127;
    f2.target = -(percPower * 0.01f) * 127;

    pros::delay(2500);


    for (int i = 0; i < disc; i++) {
        i1.target = -127;
        pros::delay(800 / 3);
        i1.target = 0;
        pros::delay(800 / 3);        
    }
    
    f1.target = 0;
    f2.target = 0;
}

void calculateDrift() {

}

void startAuto2() {
    
}

void startAuto3() {
    if (color == 1) {
        Task t(forwardDist, (void*)20);
        t1.target = 127;
        c::delay(180);
        t1.target = 0;
    } else {
        Task t(forwardDist, (void*)20);
        t1.target = 127;
        c::delay(180);
        t1.target = 0;
    }

    backDist(18);
    rotateClockwise(125);
    shoot(2, 85);
}
