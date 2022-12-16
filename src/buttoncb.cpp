#include "display/lv_core/lv_obj.h"
#include "display/lv_objx/lv_label.h"
#include "main.h"
#include "globals.hpp"
#include "buttoncb.hpp"
#include "pros/rtos.hpp"
#include <exception>
#include <string>
#include <iostream>
#include <thread>

// Button callbacks

lv_res_t keypad_back(lv_obj_t * btn) {

    lv_obj_set_hidden(mainScreen, false);
    lv_obj_set_hidden(manualScreen, true);

    passcode = "";

    return LV_RES_OK;
}

lv_res_t keypad_button(lv_obj_t * btn) {
    uint8_t id = lv_obj_get_free_num(btn);

    passcode += std::to_string(id - ((int) l.size() * 2) - 5);
    if (passcode == "01000111" || passcode == "185") {
        lv_obj_set_hidden(keypad, true);
        lv_obj_set_hidden(mainScreen, false);
        unlocked = true;
    }

    return LV_RES_OK;
}

lv_res_t toggle_button(lv_obj_t * btn) {

    return LV_RES_OK;
}

lv_res_t start3(lv_obj_t * btn) {

    spawn = 3;
    lv_label_set_text(autoInfo, "Roller Start");
    lv_obj_set_hidden(buttonBG, true);

    return LV_RES_OK;
}

lv_res_t startTest(lv_obj_t *) {
    pros::delay(1000);
    t1.target = 127;
    pros::delay(1000);
    t1.target = 0;
    pros::delay(1000);
    t1.target = 127;
    pros::delay(500);
    t1.target = 0;
    pros::delay(500);

    pros::delay(1000);
    t1.target = -127;
    pros::delay(1000);
    t1.target = 0;
    pros::delay(1000);
    t1.target = -127;
    pros::delay(500);
    t1.target = 0;
    pros::delay(500);

    i1.target = 127;
    pros::delay(1000);
    i1.target = 0;
    pros::delay(1000);
    i1.target = 127;
    pros::delay(500);
    i1.target = 0;
    pros::delay(1000);

    f1.target = 127;
    f2.target = -127;
    pros::delay(1000);
    f1.target = 0;
    f2.target = 0;
    pros::delay(1000);
    f1.target = 127;
    f2.target = -127;
    pros::delay(500);
    f1.target = 0;
    f2.target = 0;
    pros::delay(1000);
    


    pros::delay(1500);
    printToConsole("Done! Luv u dad");

    return LV_RES_OK;
}
lv_res_t startAuto(lv_obj_t * btn) {
    if (spawn == 3) {
        Task t(startAuto3);
	} else if (spawn == 2) {
		Task t(startAuto2);
	} else {
		printToConsole("I let u down dad ☹");
	}
    return LV_RES_OK;
}


lv_res_t start2(lv_obj_t * btn) {

    spawn = 2;

    lv_label_set_text(autoInfo, "Non Roller Start");
    lv_obj_set_hidden(buttonBG, true);

    return LV_RES_OK;
}

lv_res_t red(lv_obj_t * btn) {
    color = 1;

    return LV_RES_OK;
}
lv_res_t blue(lv_obj_t * btn) {
    color = 2;
    return LV_RES_OK;
}

lv_res_t btn_click_action(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn); 

    //std::cout << id << std::endl;
    if (id == 2) {
        lv_obj_set_hidden(mainScreen, true);
        lv_obj_set_hidden(configScreen, false);

    }

    if (id == 3) {
        lv_obj_set_hidden(mainScreen, false);
        lv_obj_set_hidden(configScreen, true);
    }

    if (id == 1) {
        lv_obj_set_hidden(mainScreen, true);
        lv_obj_set_hidden(manualScreen, false);
    }
    if (id == 0) {
        lv_obj_set_hidden(mainScreen, true);
        lv_obj_set_hidden(autoScreen, false);
        inAuto = true;
    }

    return LV_RES_OK;
}
lv_res_t config_button(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn); 

    float mappedIndex = floor((id-3.5)/2);
    std::string mappedS = mapKeys[mappedIndex]; 


    if (id % 2 == 1) {
        setConfig(mappedS, getConfig(mappedS) + 1);

        lv_label_set_text(&l[mappedIndex], std::to_string(getConfig(mapKeys[mappedIndex])).c_str());
        lv_label_set_text(&l[mappedIndex], std::to_string(getConfig(mapKeys[mappedIndex])).c_str());
    } else {
        setConfig(mappedS, getConfig(mappedS) - 1);

        lv_label_set_text(&l[mappedIndex], std::to_string(getConfig(mapKeys[mappedIndex])).c_str());
    }

    return LV_RES_OK;
}

lv_res_t backAuto(lv_obj_t * btn) {

        lv_obj_set_hidden(mainScreen, false);
        lv_obj_set_hidden(autoScreen, true);
        // inAuto = false;

    return LV_RES_OK;
}