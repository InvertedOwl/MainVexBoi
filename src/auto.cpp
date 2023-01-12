#include "main.h"
#include "globals.hpp"
#include "pros/adi.h"
#include "pros/ext_adi.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <exception>
#include <string>




void ldSet(int val) {
    l1.target = val;
    l3.target = val;
    l2.target = val;
}
void rdSet(int val) {
    r1.target = val;
    r2.target = val;
    r3.target = val;
}

float fix180(float heading) {
    float result = heading;

    if(heading < -180) {
        result += 360;
    } else if(heading > 180) {
        result -= 360;
    }

    return result;
}

void stop() {
    r1.target = -0;
    r2.target = -0;
    r3.target = -0;
    l1.target = 0;
    l2.target = 0;
    l3.target = 0;

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


void forwardDist(int mmDist) {
    if (!mmDist) {
        mmDist = 20;
    }

    rdSet(-127);
    ldSet(127);

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
}

void forwardDist(void* mmDi) {

    int mmDist = (int)mmDi;

    rdSet(-127);
    ldSet(127);

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
}

void backDist(int mmDist) {


    rdSet(127);
    ldSet(-127);

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
}

void rotateClockwise(int degrees, bool forceBad = false) {
    float current = imu.get_heading();
    float target = current + degrees;

    float error = target-current;

    if (forceBad) {
        float degreesHalved = degrees/2.0f;

        float motorSpeed = 127 * (float) degrees / abs(degrees);
        rdSet(motorSpeed);
        ldSet(motorSpeed);  // Possibly flip
        

        while (std::abs(error) > 1) {

            current = imu.get_heading();

            error = target-current;

            // 62-ish TPS
            pros::delay(16);
        }
        rotateClockwise(degreesHalved);
        return;
    }


    float kP = 1.5f;
    float kD = 0.085f;

    error = fix180(error);

    float deriv;
    float lastError = error;

    // PD loop
    while (std::abs(error) > 1) {
        current = imu.get_heading();
		lv_chart_set_next(PIDchart, PIDSeries, current);		
        lv_chart_set_next(PIDchart, TargetSeries, target);		

        error = target-current;
        error = fix180(error);

        deriv = error-lastError;
        lastError = error;

        float out = (error * kP) + (deriv * kD);

        rdSet(out);
        ldSet(out);

        // 62-ish TPS
        pros::delay(16);
    }
    stop();
}

void shoot(int disc, int percPower) {
    f1.target = (percPower * 0.01f) * 127;
    f2.target = -(percPower * 0.01f) * 127;

    // Speed up flywheel
    pros::delay(2500);

    // Indexer per disc
    for (int i = 0; i < disc; i++) {
        i1.target = -127;
        pros::delay(300);
        i1.target = 0;
        pros::delay(250);        
    }
    
    f1.target = 0;
    f2.target = 0;
}

void intakeOn() {
    t1.target = 127;
}

// Function for expansion is AUTOBOTS ROLL OUT!

void getRoller() {
    Task t(forwardDist, (void*)20);
    t1.target = 127;
    c::delay(180);
    t1.target = 0;
}

void autoSkills() {
    // Roller 1
    getRoller();
    backDist(20);    
    rotateClockwise(-225, true);
    intakeOn();
    forwardDist(100);
    rotateClockwise(-45);
    forwardDist(20);
    
    // Maybe turn off intake between shots?

    // Roller 2
    getRoller();
    backDist(20);
    rotateClockwise(90);
    forwardDist(200);
    shoot(3, 100);
    rotateClockwise(45);
    forwardDist(1000);
    rotateClockwise(-90);
    shoot(3, 100);
    rotateClockwise(-90);
    forwardDist(2000);
    rotateClockwise(45);
    shoot(3, 100);
    rotateClockwise(90);
    forwardDist(100);
    
    // Roller 3
    getRoller();


}

void startAuto2() {
    imu.reset();
    shoot(2, 75);
    forwardDist(508);
    rotateClockwise(90);
    getRoller();
}

void startAuto3() {
    getRoller();
    backDist(18);
    rotateClockwise(90);
    shoot(2, 85);

}
