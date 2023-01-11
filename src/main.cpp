#include "main.h"
#include "globals.hpp"
#include <algorithm>
#include <chrono>
#include <exception>
#include <string>
#include "MotorH.hpp"

using namespace pros;

int sol = 127;

bool unlocked = false;

//derek fooling around with forking for education purposes

void updateMotor() {
	while (true) {
		f1.tick();
		f2.tick();

		i1.tick();

		t1.tick();

		r1.tick();
		r2.tick();
		r3.tick();

		l1.tick();
		l2.tick();
		l3.tick();

		delay(20);
	}
}

void initialize() {
	std::cout << "Working";

	initConfig();
	initGui();

	master.rumble(".-");

	Task t(updateMotor);

	f1.motor->set_brake_mode(E_MOTOR_BRAKE_COAST);
	f2.motor->set_brake_mode(E_MOTOR_BRAKE_COAST);
	i1.motor->set_brake_mode(E_MOTOR_BRAKE_COAST);
	t1.motor->set_brake_mode(E_MOTOR_BRAKE_COAST);

	//std::cout << "Ready!" << std::endl;
}

void disabled() {
	std::cout << "Goodbye world!" << std::endl;
}

void competition_initialize() {}

void autonomous() {
	if (spawn == 3) {
		startAuto3();
	} else if (spawn == 2) {
		startAuto2();
	} else if (spawn == 4) {
		autoSkills();
	} else {
		printToConsole("HELP DAD HELP");
	}
}


void opcontrol() {
	bool lowerLast = false;
	
	// Loop
	while (true) {
		master.clear_line(1);
		master.set_text(1, 0, std::to_string(power));

		printToConsole(std::to_string(imu.get_heading()));

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
			if (power < 100) {
				power += 20;
			}
		}
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			if (power > 0) {
				power -= 20;
			}
		}

		// // Vision
		// vision_object_s_t obj = sensor.get_by_sig(0, 1);
    	// lv_obj_set_pos(visionResponse, obj.left_coord, obj.top_coord);
    	// lv_obj_set_size(visionResponse, obj.width, obj.height);

		// Master Analogs
		int lefty = (master.get_analog(ANALOG_LEFT_Y));
		int righty = (master.get_analog(ANALOG_RIGHT_Y));
		int leftx = (master.get_analog(ANALOG_LEFT_X));
		int rightx = (master.get_analog(ANALOG_RIGHT_X));


		// Deadzones
		if (std::abs(lefty) < getConfig("dzly")) {
			lefty = 0;
		}
		if (std::abs(righty) < getConfig("dzry")) {
			righty = 0;
		}
		if (std::abs(leftx) < getConfig("dzlx")) {
			leftx = 0;
		}
		if (std::abs(rightx) < getConfig("dzrx")) {
			rightx = 0;
		}


		// Deadzone recalc
		float posneg = 0;
		float posnegthesequal = 0;

		posneg = (float(righty) / abs(righty));

		float maxry = ((127.0f * posneg) - (getConfig("dzry")) * posneg);
		float valry = (righty - (getConfig("dzry") * posneg));
		float fixedry = valry/maxry * 127.0f * posneg;

		posnegthesequal = (float(lefty) / abs(lefty));

		float maxly = ((127.0f * posnegthesequal) - (getConfig("dzly")) * posnegthesequal);
		float vally = (lefty - (getConfig("dzly") * posnegthesequal));
		float fixedly = vally/maxly * 127.0f * posnegthesequal;
		

		lv_label_set_text(manualText, ("Left Y: " + std::to_string(lefty) + "\nLeft X: " + std::to_string(leftx) + "\nRight Y: " + std::to_string(righty) + "\nRight X: " + std::to_string(rightx) + "\nFixedly: " + std::to_string(fixedly) + "\nFixedry: " + std::to_string(fixedry) + "\nCurrent time: " + std::to_string(std::time(nullptr)) +
		"\nF1: " + std::to_string(f1.current) + " F1=: " + std::to_string(f1.motor->get_actual_velocity()) + "\nF2: " + std::to_string(f2.current)
		).c_str());


		// Main loop
		r1.target = -fixedry;
		r2.target = -fixedry;
		r3.target = -fixedry;
		l1.target = fixedly;
		l2.target = fixedly;
		l3.target = fixedly;



		// Flywheel + indexer controls
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {

			if (f1.motor->get_actual_velocity() > 170 && !lowerLast) {
				lowerLast = true;

				master.rumble("-");
			}
			if (f1.motor->get_actual_velocity() < 170) {
				lowerLast = false;
			}

			f1.target = 96 + (32 * (0.01f * power));
			f2.target = -96 + (-32 * (0.01f * power));
		} else {
			f1.target = 0;
			f2.target = 0;
		}

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			i1.target = -127;
		} else {
			i1.target = 0;
		}

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			t1.target = 127;
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			t1.target = -127;
		} else {
			t1.target = 0;
		}

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
			solenoid.set_value(true);
		} else {
			solenoid.set_value(false);
		}

		delay(20);

	}

} 