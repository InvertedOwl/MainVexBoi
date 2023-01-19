#include "main.h"
#include "globals.hpp"
#include "pros/adi.h"
#include "pros/ext_adi.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <exception>
#include <string>

/*GENERAL REFERENCE

-field tile is 2ft (609.60 millimeters) by 2ft (609.60 millimeters)
-disk is 1.5 inches (38.10 millimeters) by 1.5 inches (38.10 millimeters)
-other stuff here

*/

// Left drive set speed
void ldSet(int val) {
    l1.target = val;
    l3.target = val;
    l2.target = val;
}

// Right drive set speed
void rdSet(int val) {
    r1.target = val;
    r2.target = val;
    r3.target = val;
}

// fix to 180 deg for PID loop - rember whiteboard time
float fix180(float heading) {
    float result = heading;

    if(heading < -180) {
        result += 360;
    } else if(heading > 180) {
        result -= 360;
    }

    return result;
}

// Stops all motors
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

// Move forward a certain amount (in millimeters)? (inaccurate) :(
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

void forwardPID (int mmDi) {
        // PID Val init
    float kP = 2;
    float kD = 0.15f;
    int initalDegrees = r1.getMotorPos();
    int target = ((mmDi / 319.28) * -360) + initalDegrees;
    float error = target-initalDegrees;

    error = fix180(error);

    float deriv;
    float lastError = error;

    //kk need this explained like wut - derek
    // Its the PD loop :) - Wes

    // PD loop
    while (std::abs(error) > 6) {
        initalDegrees = imu.get_heading();


        // Get error and fix to (-180 | 180)
        error = target-initalDegrees;
        // error = fix180(error);

        // Calculate the derivitive
        deriv = error-lastError;
        lastError = error;

        // Use values Derivitive and Proportional to get output value
        float out = (error * kP) + (deriv * kD);

        // Set values
        rdSet(out);
        ldSet(-out);

        // 62-ish TPS
        pros::delay(16);
    }
    stop();
}

// Same as forwardDist, just allows for threading
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

// Move back a certain distance in millimeters? (also innaccurate) :(
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

// New rotate function - allows non-optimised rotation
void rotateClockwise(int degrees, bool forceBad = false, float prop = 1.6f) {
    // Init values
    float current = imu.get_heading();
    float target = current + degrees;
    float error = target-current;

    // Go half rotation at full speed IF forcebad
    if (forceBad) {
        // Degrees to turn
        float degreesHalved = degrees/2.0f;

        target = current + degreesHalved;
            
        if (target < 0 && forceBad) {
            target += 360;
        }
        if (target > 360 && forceBad) {
            target -= 360;
        }
        
        // Calc speed for motors (Just gets the direction needed)
        float motorSpeed = 127 * (float) degrees / abs(degrees);
        rdSet(motorSpeed);
        ldSet(motorSpeed);
        
        // Wait until at at least half :)
        while (std::abs(error) > 10) { // Change error error? 10*>5

            current = imu.get_heading();

            error = target-current;

            // 62-ish TPS
            pros::delay(16);
        }
        rotateClockwise(degreesHalved);
        return;
    }

    // PID Val init
    float kP = prop;
    float kD = 0.15f;

    error = fix180(error);

    float deriv;
    float lastError = error;

    //kk need this explained like wut - derek
    // Its the PD loop :) - Wes

    // PD loop
    while (std::abs(error) > 6) {
        current = imu.get_heading();
        // Plot PD but no workie
		lv_chart_set_next(PIDchart, PIDSeries, current);		
        lv_chart_set_next(PIDchart, TargetSeries, target);		

        // Get error and fix to (-180 | 180)
        error = target-current;
        error = fix180(error);

        // Calculate the derivitive
        deriv = error-lastError;
        lastError = error;

        // Use values Derivitive and Proportional to get output value
        float out = (error * kP) + (deriv * kD);

        // Set values
        rdSet(out);
        ldSet(out);

        // 62-ish TPS
        pros::delay(16);
    }
    stop();
}

// Shoot amount (int disc) of disks, shoot at [int percPower] percent of flywheel power
void shoot(int disc, int percPower) {
    f1.target = (percPower * 0.01f) * 127;
    f2.target = -(percPower * 0.01f) * 127;

    // Speed up flywheel
    pros::delay(2000);

    // Indexer per disc
    for (int i = 0; i < disc; i++) {
        
        

        // Wait until flywheel is at speed (using power)
        if (percPower > 65) {
            while (f1.motor->get_actual_velocity() < 200 * (( 96 + (32 * (0.01f * percPower))) / 127.0f)) {
                pros::delay(16);
            }
            printToConsole(std::to_string(f1.motor->get_actual_velocity()));
        } else {
            pros::delay(500);
        }



        // Index one disc
        i1.target = -127;
        pros::delay(100);
        while (!limitIndexer.get_new_press()) {
            pros::delay(16);
        }

        i1.target = 0;
        pros::delay(100);
    }
    
    //spin back down
    f1.target = 0;
    f2.target = 0;
}

//turn intake on
void intakeOn() {
    t1.target = 127;
}

// Function for expansion is AUTOBOTS ROLL OUT!

// Gets roller directly in front (Move forward and roller at the same time)
void getRoller() {
    // Go forward and KEEP GOING without stopping the thread
    Task t(forwardDist, (void*)20);
    t1.target = 127;
    c::delay(180);
    t1.target = 0;
}

// Goblin mode
void autoSkills() {
    // Roller 1
    getRoller();
    rotateClockwise(-15);
    backDist(80);    
    forwardDist(30);
    rotateClockwise(-225, true);
    intakeOn();
    forwardDist(390);
    rotateClockwise(-45);
    forwardDist(150);
    
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

// Right auto
void startAuto2() {
    if (!aggressive) {

        shoot(2, 65);
        forwardDist(516);
        rotateClockwise(90);
        forwardDist(20);
        getRoller();
    } else {
        forwardDist(492);
        rotateClockwise(90);
        forwardDist(20);
        getRoller();

        backDist(18);
        // rotateClockwise(19, false, 2.5f);
        shoot(2, 91);

        // rotateClockwise(18, false, 2.36f);
        // shoot(2, 91);
        // rotateClockwise(-90-18);
        // forwardDist(508);
        // rotateClockwise(90);
        // forwardDist(65);
        // getRoller();
    }
}

// Left auto
void startAuto3() {
    if (!aggressive) {
        // PASSIVE
        getRoller();
        backDist(18);
        rotateClockwise(85);
        shoot(2, 65);
    } else {
        //AGGRESSIVE
        getRoller();
        backDist(18);
        rotateClockwise(-18, false, 1.85f);
        shoot(2, 91);
    }
}

/*derek testing auto - WARNING - likely very bad

outline for autonomous skills:

-have robo start somewhere
-turn until spot disk
 upon spotting: 
	-immediately stop, drive towards disk until unknown distance from disk
	-pick up disk
	-change disk counter by +1? any way to tell if it picks up an unintentional amount of disks?
-create coord system so that when it has enough disks it can go to place to shoot

EVENTUALLY:
prolly gonna want a thing to check low goals for disks maybe
have a way to make sure that one basket doesnt get too full?

*/

void forwardSeconds(void* seconds) {
    double second = *((double*)seconds);
    rdSet(-127);
    ldSet(127);
    pros::delay(second * 1000);
    rdSet(0);
    ldSet(0);
}

void backwardSeconds(void* seconds) {
    double second = *((double*)seconds);
    rdSet(127);
    ldSet(-127);
    pros::delay(second * 1000);
    rdSet(0);
    ldSet(0);
}

void forwardMeters(void* meters) {
    //double metersToMove = *((double*)meters);
    double secondsToMove = *((double*)meters) / velocity;
    forwardSeconds(&secondsToMove);
}

void backwardMeters(void* meters) {
    //double second = *((double*)seconds);
    //backwardSeconds(metersToMove / velocity);
    double secondsToMove = *((double*)meters) / velocity;
    backwardSeconds(&secondsToMove);
    
}

//finds velocity of robot
void calibrate() {
    double numSeconds = 2.0;
    double rawVelocity = 0;
    pros::c::imu_accel_s_t accel = imu.get_accel(); //idk just copied wiki code
    for(double i = (double)numSeconds; i > 0; i - .1) {
        rawVelocity += accel.x;
        pros::delay(100);
    }
    
    backwardSeconds((void*) &numSeconds); //move back to original position

    velocity = rawVelocity / (10 * numSeconds); //acceleration gathered every 1/10s, not 1s --> divide by 10 to get actual velocity - multiply by number of seconds to get meters a *second*
}