#include "main.h"
#include <map>
#include <string>
#include <vector>
#include "MotorH.hpp"
#include "pros/imu.hpp"


using namespace pros;

#ifndef GLOBALS
#define GLOBALS

// - Vex stouf -
extern Controller master;
extern MotorH l1, l2, l3, r1, r2, r3;
extern MotorH f1, f2;
extern MotorH i1, t1;
extern bool unlocked;
extern Vision sensor;
extern ADIDigitalOut solenoid;
extern ADIDigitalOut solenoid2;
extern int power;
extern Imu imu;

// - Config -
extern std::vector<std::string> mapKeys;
extern int getConfig(std::string);
extern void setConfig(std::string, int);

// - Init -
extern void initGui();
extern void initConfig();


// - Auto -
extern void startAuto2();
extern void startAuto3();
extern void autoSkills();
extern bool inAuto;
extern c::adi_gyro_t gyro;
extern int spawn;
extern int color;


// - GUI -
extern void createButton(int, int, int, int, lv_action_t, lv_obj_t *, const char *);
extern void printToConsole(std::string);
extern std::vector<lv_obj_t> l;

extern lv_obj_t * console;
extern lv_obj_t * consoleText;
extern lv_obj_t * window;
extern lv_obj_t * mainScreen;
extern lv_obj_t * configScreen;
extern lv_obj_t * manualScreen;
extern lv_obj_t * autoScreen;
extern lv_obj_t * keypad;
extern lv_obj_t * manualText;
extern lv_obj_t * visionResponse;
extern lv_obj_t * visionResponse2;

extern lv_obj_t * buttonBG;

extern lv_obj_t * autoInfo;

extern lv_obj_t * lastPage;
extern lv_obj_t * nextPage;

extern std::string passcode;

extern lv_obj_t * PIDchart;
extern lv_chart_series_t * PIDSeries;
extern lv_chart_series_t * TargetSeries;

extern lv_obj_t * calibrateButton;

#endif