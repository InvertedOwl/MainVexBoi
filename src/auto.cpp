#include "main.h"
#include "globals.hpp"
#include "pros/adi.h"
#include "pros/ext_adi.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <cmath>
#include <exception>
#include <string>

Rotation flywheelSpeed(1);

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
void forwardDist(int mmDist, int speed = 127) {
    if (!mmDist) {
        mmDist = 20;
    }

    rdSet(-speed);
    ldSet(speed);

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
void rotateClockwise(int degrees, bool forceBad = false, float prop = 1.9f) {
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
        float kI = 0.009f;
        float integral = 0;

        error = fix180(error);

        float deriv;
        float lastError = error;

        // PD loop with I term
    while (std::abs(error) > 6) {
        current = imu.get_heading();
        // Plot PD but no workie
        lv_chart_set_next(PIDchart, PIDSeries, current);
        lv_chart_set_next(PIDchart, TargetSeries, target);

        // Get error and fix to (-180 | 180)
        error = target-current;
        error = fix180(error);

        // Calculate the derivative
        deriv = error-lastError;
        lastError = error;

        // Calculate integral

        // Use values Derivative, Proportional and Integral to get output value
        if (std::abs(error) < 15) {
            integral += error;

        }
        float out = (error * kP) + (deriv * kD) + (integral * kI);

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
    disc += 1;
    f1.target = (percPower * 0.01f) * 127;
    f2.target = (percPower * 0.01f) * 127;

    // Speed up flywheel
    pros::delay(2000);

    // Indexer per disc
    for (int i = 0; i < disc; i++) {
        
        

        // Wait until flywheel is at speed (using power)
        if (percPower > 65) {
            int count = 0;
            while (f1.motor->get_actual_velocity() < 190 * (0.01f * percPower)) {
                if (count == 60 * 0.2f) {
                    break;
                }
                count += 1;
                pros::delay(16);
            }
            printToConsole(std::to_string(f1.motor->get_actual_velocity()));
        } else {
            pros::delay(500);
        }



        // Index one disc
        i1.target = 127;
        pros::delay(100);
        while (!limitIndexer.get_new_press()) {
            pros::delay(100);
        }

        i1.target = 0;
        pros::delay(100);
    }
    
    //spin back down
    f1.target = 0;
    f2.target = 0;
}

void shoot2(int disc, double propPower) {
    disc += 1;
    //f1.target = (percPower * 0.01f) * 127;
    //f2.target = (percPower * 0.01f) * 127;

    //new math

    f1.target = propPower * 127;
    f2.target = propPower * 127;


    // Speed up flywheel
    pros::delay(2000);

    // Indexer per disc
    /*for (int i = 0; i < disc; i++) {
        
        

        // Wait until flywheel is at speed (using power)
        if (percPower > 65) {
            int count = 0;
            while (f1.motor->get_actual_velocity() < 190 * (0.01f * percPower)) {
                if (count == 60 * 0.2f) {
                    break;
                }
                count += 1;
                pros::delay(16);
            }
            printToConsole(std::to_string(f1.motor->get_actual_velocity()));
        } else {
            pros::delay(500);
        }



        // Index one disc
        i1.target = 127;
        pros::delay(100);
        while (!limitIndexer.get_new_press()) {
            pros::delay(100);
        }

        i1.target = 0;
        pros::delay(100);
    }
    */
    for (int i = 0; i < disc; i++) {
        while(flywheelSpeed.get_velocity() > -14650) {
            pros::delay(100);
            printToConsole("i waited");
        }
        printToConsole(std::to_string(flywheelSpeed.get_velocity()));
        //pros::delay(600);
        
        //printToConsole("i tried to wait");

         // Index one disc
        i1.target = 127;
        //pros::delay(100);
        while (!limitIndexer.get_new_press()) {
            pros::delay(80);
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
void getRoller (int time = 180) {
    // Go forward and KEEP GOING without stopping the thread
    Task t(forwardDist, (void*)20);
    t1.target = 127;
    c::delay(time);
    t1.target = 0;
}

// Goblin mode
void autoSkills() {
    // Roller 1
    getRoller(180*2);
    backDist(20);
    rotateClockwise(-45, false, 2.3f);
    backDist(125);    
    forwardDist(50);
    rotateClockwise(-210+40, false);
    intakeOn();
    forwardDist(390);
    rotateClockwise(-45);
    forwardDist(110);
    
    // Maybe turn off intake between shots?

    // Roller 2
    getRoller(180*2);
    intakeOn();
    backDist(20);
    rotateClockwise(88);
    forwardDist(800);
    rotateClockwise(160, true);
    shoot(3, 100);

    rotateClockwise(-45);
    forwardDist(410);
    rotateClockwise(-85);
    forwardDist(800);
    rotateClockwise(70);
    backDist(100);
    shoot(3, 100);
    forwardDist(100);
    rotateClockwise(-100);
    forwardDist(500);
    rotateClockwise(80);
    forwardDist(600);
    rotateClockwise(-90);
    forwardDist(550);
    getRoller(180*2);
    backDist(50);
    rotateClockwise(-135);

    solenoid.set_value(true);
    
    // rotateClockwise(-135);
    // intakeOn();
    // forwardDist(1000);
    // rotateClockwise(-90);
    // shoot(3, 100);
    // rotateClockwise(-90);
    // forwardDist(2000);
    // rotateClockwise(45);
    // shoot(3, 100);
    // rotateClockwise(90);
    // forwardDist(100);
    
    // Roller 3
    // getRoller();


}

// Right auto
void startAuto2() {
    if (!aggressive) {

        shoot(2, 85);
        forwardDist(460);
        rotateClockwise(90);
        forwardDist(20);
        getRoller();
    } else {
        forwardDist(460);
        rotateClockwise(90, false, 1.8f);
        forwardDist(20);
        getRoller();

        backDist(18);
        // rotateClockwise(19, false, 2.5f);
        shoot(2, 83);

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
    // 0
    // - 18
    // - 131
    // 131 + 83

    if (!aggressive) {
        // PASSIVE
        getRoller();
        backDist(18);
        rotateClockwise(90-10);
        shoot(2, 90);
        rotateClockwise(135);
        forwardDist(350-100);
        intakeOn();
        forwardDist(700+100, 32);
        rotateClockwise(-120);
        shoot(3, 70);
        // rotateClockwise(84, false, 2.0f);
        // shoot(3, 74);

    } else {
        //AGGRESSIVE
        /*f1.target = (78 * 0.01f) * 127;
        f2.target = (78 * 0.01f) * 127;
        getRoller();
        backDist(18);
        rotateClockwise(-18, false, 2.2f);
        shoot(2, 78);
        f1.target = (74 * 0.01f) * 127;
        f2.target = (74 * 0.01f) * 127;
        // pros::delay(2000);
        rotateClockwise(-113, false, 1.9f);
        forwardDist(350);
        intakeOn();
        forwardDist(700, 32);
        rotateClockwise(87, false, 2.0f);
        shoot(3, 74);
        */
        
        /*f1.target = .75 * 127;
        f2.target = .75 * 127;
        double fieldRollerProp = 2;
        getRoller(280);
        backDist(30);
        rotateClockwise(-15, false);
        printToConsole("i rotated");
        shoot2(2, .80);
        */
        rotateClockwise(-15, false);
        shoot2(2, .85);

        pros::delay(200);

        rotateClockwise(15, false);
        forwardDist(15);
        getRoller(360);

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
    double secondsToMove = *((double*)meters) / velocity;    forwardSeconds(&secondsToMove);
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