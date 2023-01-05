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

//what?? - derek
void ldSet(int val) {
    l1.target = val;
    l3.target = val;
    l2.target = val;
}
//what?? - derek 
void rdSet(int val) {
    r1.target = val;
    r2.target = val;
    r3.target = val;
}
//adjusting for drift? - derek
float fix180(float heading) {
    float result = heading;

    if(heading < -180) {
        result += 360;
    } else if(heading > 180) {
        result -= 360;
    }

    return result;
}
//stops all functions? - derek
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

//move forward a certain amount (in millimeters)? - derek
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

//kk wat this me no c++ - why is there a void*? does that just mean no type? - derek
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

//move back a certain distance in millimeters? - derek
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

//rotate clockwise a certain amount of degrees - derek
void rotateClockwise(int degrees) {

    float kP = 1.5f;
    float kD = 0.085f;

    float current = imu.get_heading(); //current = heading before turn
    float target = current + degrees; //target = take heading before turn and add desired degrees to turn to get target position

    float error = target-current; //what
    error = fix180(error); //hmm ok need to ask about this - this is probably in relation to drift??

    float deriv;
    float lastError = error;

    //kk need this explained like wut - derek
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
}

//shoot amount (int disc) of disks, shoot at [int percPower] percent of flywheel power
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
    
    //spin back down
    f1.target = 0;
    f2.target = 0;
}

//assuming this means that when in the desired position, go forward and roll the roller in (makes it desired team color) - derek
void getRoller() {
    Task t(forwardDist, (void*)20);
    t1.target = 127;
    c::delay(180);
    t1.target = 0;
}

//when autonomous is set in 2 mode (position away from roller i think):
void startAuto2() {
    imu.reset(); //reset gyro
    shoot(2, 75); //shoot 2 (preloads) at 75% power
    forwardDist(508); //go forward 508 mm (1.67 ft? almost full tile)
    rotateClockwise(90); //rotate 90 deg
    getRoller(); //roll the roller lol
}

//when autonomous is set in 3 mode (position in front of roller i think):
void startAuto3() {
    getRoller(); //roll the roller pls
    backDist(18); //go back 18mm
    rotateClockwise(90); //rotate 90 deg
    shoot(2, 85); //shoot 2 discs at 85% power

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

pros::Optical optical_sensor(/*figure out port lol*/);
pros::c::optical_rgb_s_t rgb_value;    //perhaps not nescessary / actually useful to use

int numDisks = 0;   //number of disks currently stored in robot

void senseDisk() {
    //if more complicated scan is needed
}

void eatDisk() {
    t1.target = 127;    //power up intake
    forwardDist(38.10);    //move forward 38.10 mm (length of disk) - may need tuning
}

void startAutoSkillz() {
    while(true) { //autonomous skills loop

        //do i need to use a variable for optical sensor rgb input? dont think so
        //rgb_value = optical_sensor.get_rgb();

        if(optical_sensor.get_rgb() == "yellow") { //if the optical sensor detects yellow

            //idea: maybe need two optical sensors? one to scan for farther disks and have the robo start driving forward, one to detect when its close?
            //if thats too much then maybe have bobot just traverse path and use the color sensor to detech how many disks the robot has?

            while(optical_sensor.get_proximity() > /*idk this needs to be tested*/ 50) {    //move forward by 1/10 of a field tile until right in front of disk
                forwardDist(60.96);
            }
            
            eatDisk();

            //add a delay here if issues arise with eating while driving
        }

        // add next step - spin until disk spotted, then repeat above
    }
}

/*MAIN LOOP:
    -see if disk detected at start
    -if so, start heading towards - stop right in front - spin up intake - move forward and eat - increase disk # var in robot
    -start spin detection
    -once there are three disks use that super cool coord system you made to get to shooting position and do so - then get away from shooting position and start loop once more
