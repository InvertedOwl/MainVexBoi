#include "display/lv_core/lv_obj.h"
#include "display/lv_core/lv_style.h"
#include "display/lv_misc/lv_color.h"
#include "display/lv_objx/lv_bar.h"
#include "display/lv_objx/lv_chart.h"
#include "display/lv_objx/lv_img.h"
#include "display/lv_objx/lv_label.h"
#include "main.h"
#include "buttoncb.hpp"
#include "globals.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"

using namespace pros;

// initialize inputs/outputs
Controller master(E_CONTROLLER_MASTER);
Controller partner(E_CONTROLLER_PARTNER);
//changeed to temporary slew rate (10 -- 10)
//MotorH l1(1, 10), l2(2, 10), l3(3, 10), r1(11, 10), r2(5, s), r3(6, 25);
MotorH l1(19, 25), l2(16, 25), l3(3, 25), r1(11, 25), r2(14, 25), r3(6, 25);
MotorH f1(18, 10, 4), f2(17, 10, 4);
MotorH i1(12, 25);
MotorH t1(13, 25);
ADIDigitalOut solenoid(8);
ADIDigitalOut solenoid2(7);
double velocity = 0;

ADIDigitalIn limitIndexer(6);

std::string constoleT = "";
pros::Vision sensor (11);
c::adi_gyro_t gyro = c::adi_gyro_init(1, 0.1);
bool inAuto = false;
Imu imu(15);
int color = 0;
bool aggressive = false;

int power = 20;

lv_obj_t * console;
lv_obj_t * consoleText;
lv_obj_t * window;
lv_obj_t * mainScreen;
lv_obj_t * configScreen;
lv_obj_t * manualScreen;

lv_obj_t * autoScreen;
lv_obj_t * keypad;
lv_obj_t * manualText;
lv_obj_t * buttonBG;

lv_obj_t * autoInfo;

lv_obj_t * lastPage;
lv_obj_t * nextPage;
lv_obj_t * visionResponse;
lv_obj_t * visionResponse2;

lv_obj_t * calibrateButton;

lv_obj_t * PIDchart;
lv_chart_series_t * PIDSeries;
lv_chart_series_t * TargetSeries;


std::string passcode;

lv_style_t myButtonStyleREL; //relesed style
lv_style_t myButtonStylePR; //pressed style

int spawn = 0;

int page = 0;

std::vector<lv_obj_t> l;

void printToConsole(std::string s) {
    constoleT += s + "\n";
    lv_label_set_long_mode(consoleText, LV_LABEL_LONG_BREAK);
    lv_label_set_text(consoleText, constoleT.c_str());
    lv_obj_align(consoleText, console, LV_ALIGN_IN_BOTTOM_MID, 0, 0);

}

void initGui() {
    std::cout << "Initialized motors and controller... " << std::endl;

    // Background
    window = lv_obj_create(lv_scr_act(), NULL);
    lv_obj_set_size(window, 480, 272);                       
    lv_obj_set_pos(window, 0, 0);

    static lv_style_t style;
    lv_style_copy(&style, &lv_style_pretty);
    style.body.main_color = LV_COLOR_BLACK;
    style.body.grad_color = LV_COLOR_BLACK;
    lv_obj_set_style(window, &style);

    mainScreen = lv_obj_create(window, NULL);
    lv_obj_set_size(mainScreen, 480, 272);                       
    lv_obj_set_pos(mainScreen, 0, 0);
    lv_obj_set_style(mainScreen, &style);
    lv_obj_set_hidden(mainScreen, false);

    lv_obj_t * back = lv_img_create(mainScreen, NULL);
    lv_obj_set_size(back, 1000, 1000);                       
    lv_obj_set_pos(back, 0, 0);

    LV_IMG_DECLARE(logobg);
    lv_img_set_src(back, &logobg);

    createButton(0-10, 100, 150, 35, btn_click_action, mainScreen, "Auto");
    createButton(-155-10, 100, 150, 35, btn_click_action, mainScreen, "Manual");
    createButton(155-10, 100, 150, 35, btn_click_action, mainScreen, "Config");


    console = lv_obj_create(mainScreen, NULL);
    lv_obj_set_pos(console, 280+50 + 25, 0);
    lv_obj_set_size(console, 150 - 25, 170);
    
    static lv_style_t consoleStyle;
    lv_style_copy(&consoleStyle, &lv_style_pretty);
    consoleStyle.body.main_color = LV_COLOR_GRAY;
    consoleStyle.body.grad_color = LV_COLOR_GRAY;
    lv_obj_set_style(console, &consoleStyle);

 

    static lv_style_t lab;
    lv_style_copy(&lab, &lv_style_plain_color);
    lab.text.color = LV_COLOR_WHITE;
    lab.body.main_color = LV_COLOR_WHITE;
    lab.body.grad_color = LV_COLOR_WHITE;

    consoleText = lv_label_create(console, NULL);
    lv_label_set_long_mode(consoleText, LV_LABEL_LONG_BREAK);
    lv_label_set_text(consoleText, constoleT.c_str());
    lv_obj_set_size(consoleText, 150 - 33, 0);
    
    lv_obj_align(consoleText, console, LV_ALIGN_IN_BOTTOM_MID, 35, 0);


    // Creates buttons and screens for config
    configScreen = lv_obj_create(window, NULL);
    lv_obj_set_size(configScreen, 480, 272);                       
    lv_obj_set_pos(configScreen, 0, 0);
    lv_obj_set_style(configScreen, &style);
    createButton(-480/2 + (150/2), 270/2 - 40, 100, 35, btn_click_action, configScreen, "Back");

    // Creates buttons and screens for config
    autoScreen = lv_obj_create(window, NULL);
    lv_obj_set_size(autoScreen, 480, 272);                       
    lv_obj_set_pos(autoScreen, 0, 0);
    lv_obj_set_style(autoScreen, &style);
    lv_obj_set_hidden(autoScreen, true);

    autoInfo = lv_label_create(autoScreen, NULL);
    // lv_label_set_long_mode(manualText, LV_LABEL_LONG_BREAK);
    lv_label_set_text(autoInfo, "...");
    lv_obj_set_size(autoInfo, 150, 0);
    lv_obj_set_style(autoInfo, &lab);

    PIDchart = lv_chart_create(autoScreen, NULL);
    lv_obj_set_size(PIDchart, 200, 150);
    lv_obj_align(PIDchart, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_chart_set_type(PIDchart, LV_CHART_TYPE_LINE);   /*Show lines and points too*/

    createButton(0, 0, 200, 35, setAgressive, autoScreen, "AGGRESSIIIVEE");

    PIDSeries = lv_chart_add_series(PIDchart, LV_COLOR_BLACK);
    TargetSeries = lv_chart_add_series(PIDchart, LV_COLOR_NAVY);
    lv_chart_set_point_count(PIDchart, 50);
    lv_chart_set_range(PIDchart, -180, 180);
    



    buttonBG = lv_obj_create(mainScreen, NULL);
    lv_obj_set_size(buttonBG, 300, 200);
    lv_obj_align(buttonBG, mainScreen, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style(buttonBG, &lv_style_pretty);


    createButton(55, 55, 100, 35, start3, buttonBG, "Set 3");
    createButton(-55, 55, 100, 35, start2, buttonBG, "Set 2");
    createButton(0, 0, 100, 35, startSkills, buttonBG, "Goblin Mode");


    // Creates buttons and labels for each item saved in config
    for (int i = 0; i < mapKeys.size(); i++) {
       // std::cout << mapKeys[i] << std::endl;

        lv_obj_t * label = lv_label_create(configScreen, NULL);
        lv_label_set_text(label, std::to_string(getConfig(mapKeys[i])).c_str());
        lv_label_set_long_mode(label, LV_LABEL_LONG_CROP);

        lv_obj_set_pos(label, 480/2 - 20, 10 + i * 40);
        lv_obj_set_size(label, 400, 200);
        
        lv_obj_t * label2 = lv_label_create(configScreen, NULL);
        lv_label_set_text(label2, (mapKeys[i]).c_str());
        lv_obj_set_pos(label2, 50, 10 + i * 40);
        lv_obj_set_size(label2, 200, 200);

        createButton(-20, i * 40 - 90, 50, 30, config_button, configScreen, "-");
        createButton(130, i * 40 - 90, 50, 30, config_button, configScreen, "+");

        lv_obj_set_style(label,&lab);
        lv_obj_set_style(label2,&lab);

        // Saves the labels so they can be updated later
        l.push_back(*label);
    }
    lv_obj_set_hidden(configScreen, true);
    
    // nextPage = createButton(480/2-45, 0, 40, 40, lv_action_t, lv_obj_t *, const char *);

    // Screen for manual    
    manualScreen = lv_obj_create(window, NULL);
    lv_obj_set_size(manualScreen, 480, 272);                       
    lv_obj_set_pos(manualScreen, 0, 0);
    lv_obj_set_style(manualScreen, &style);
    createButton(-480/2 + (150/2), 270/2 - 40, 100, 35, keypad_back, manualScreen, "Back");





    // Create keypad for manual protection;
    static lv_style_t keyback;
    lv_style_copy(&keyback, &lv_style_plain_color);
    keyback.body.main_color = LV_COLOR_GRAY;
    keyback.body.grad_color = LV_COLOR_GRAY;

    keypad = lv_obj_create(lv_scr_act(), NULL);
    lv_obj_set_size(keypad, 200, 400);
    lv_obj_align(keypad, manualScreen, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style(keypad, &keyback);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            createButton((j-1) * 55 + 37, (i-1) * 55 - 25, 50, 50, keypad_button, keypad, std::to_string((i * 3) + j).c_str());
        }
    }
    createButton((1-1) * 55 + 37, (3-1) * 55 - 25, 50, 50, keypad_button, keypad, std::to_string(9).c_str());
    lv_obj_set_hidden(manualScreen, true);

    manualText = lv_label_create(manualScreen, NULL);
    // lv_label_set_long_mode(manualText, LV_LABEL_LONG_BREAK);
    lv_label_set_text(manualText, "...");
    lv_obj_set_size(manualText, 150, 0);
    lv_obj_set_style(manualText, &lab);

    createButton(0, 0, 130, 35, startTest, manualScreen, "Start Test");

    static lv_style_t visionStyle;
    lv_style_copy(&visionStyle, &lv_style_pretty);
    visionStyle.body.main_color = LV_COLOR_BLUE;
    visionStyle.body.grad_color = LV_COLOR_BLUE;
    static lv_style_t visionStyle2;
    lv_style_copy(&visionStyle2, &lv_style_pretty);
    visionStyle2.body.main_color = LV_COLOR_RED;
    visionStyle2.body.grad_color = LV_COLOR_RED;
    lv_obj_set_hidden(keypad, true);

    visionResponse = lv_obj_create(autoScreen, NULL);
    lv_obj_set_pos(visionResponse, 0, 0);
    lv_obj_set_size(visionResponse, 0, 0);
    lv_obj_set_style(visionResponse, &visionStyle);

    visionResponse2 = lv_obj_create(autoScreen, NULL);
    lv_obj_set_pos(visionResponse2, 0, 0);
    lv_obj_set_size(visionResponse2, 0, 0);
    lv_obj_set_style(visionResponse2, &visionStyle2);


    // lv_obj_align(manualText, manualScreen, LV_ALIGN_IN_BOTTOM_MID, 0, 0);

    createButton(-480/2 + (150/2), 270/2 - 60, 100, 35, btn_click_action, configScreen, "<");
    createButton(480/2 - (150/2), 270/2 - 60, 100, 35, btn_click_action, configScreen, ">");
    createButton(-480/2 + (150/2), 270/2 - 40, 100, 35, backAuto, autoScreen, "Back");

    createButton(0, -150, 100, 35, setCalibrateTest, autoScreen, "Test Calibrate");

    std::cout << "Initialized gui... " << std::endl;
}





