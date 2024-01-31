/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\T0825640Z                                        */
/*    Created:      Tue Jun 27 2023                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <sstream>
#include <map>
using namespace vex;
using namespace std;

motor M1 = motor(PORT1, gearSetting::ratio18_1, false);
motor M2 = motor(PORT2, gearSetting::ratio18_1, false);
motor M3 = motor(PORT3, gearSetting::ratio18_1, false);
motor M4 = motor(PORT4, gearSetting::ratio18_1, false);
motor M5 = motor(PORT5, gearSetting::ratio18_1, false);
motor M6 = motor(PORT6, gearSetting::ratio18_1, false);
motor M7 = motor(PORT7, gearSetting::ratio18_1, false);
motor M8 = motor(PORT8, gearSetting::ratio18_1, false);
motor M9 = motor(PORT9, gearSetting::ratio18_1, false);
motor M10 = motor(PORT10, gearSetting::ratio18_1, false);
motor M11 = motor(PORT11, gearSetting::ratio18_1, false);
motor M12 = motor(PORT12, gearSetting::ratio18_1, false);
motor M13 = motor(PORT13, gearSetting::ratio18_1, false);
motor M14 = motor(PORT14, gearSetting::ratio18_1, false);
motor M15 = motor(PORT15, gearSetting::ratio18_1, false);
motor M16 = motor(PORT16, gearSetting::ratio18_1, false);
motor M17 = motor(PORT17, gearSetting::ratio18_1, false);
motor M18 = motor(PORT18, gearSetting::ratio18_1, false);
motor M19 = motor(PORT19, gearSetting::ratio18_1, false);
motor M20 = motor(PORT20, gearSetting::ratio18_1, false);

//USE PORT 21 for RADIO
//motor M21 = motor(PORT20, gearSetting::ratio18_1, false);
motor USE[4] = {M1, M2, M3, M4};
string NAMES[4] = {"M1", "M2", "M3", "M4"};
//map<string,motor> MOTORS;
map<double, map<string,double>> readMap;

controller H = controller(primary);
controller V = controller(partner);

bool driverControl = false;

//to_string function patch due to Mingui https://stackoverflow.com/questions/12975341/to-string-is-not-a-member-of-std-says-g-mingw
namespace std {
    template<typename T>
    std::string to_string(const T &n) {
        std::ostringstream s;
        s << n;
        return s.str();
    }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  //MOTORS["M1"] = M1;
  //MOTORS["M2"] = M2;
  //MOTORS["M3"] = M3;
  //MOTORS["M4"] = M4;
  vexcodeInit();
  
}

void countdown(){
  int counter = 3;
  while (counter < 0){
    H.Screen.clearScreen();
    H.Screen.setCursor(1,1);
    H.Screen.print(counter);
    H.rumble("-");
    task::sleep(1000);
  }
  H.Screen.clearScreen();
}

void Record(){
  if (!Brain.SDcard.isInserted()){
    Brain.Screen.print("SD CARD NOT INSERTED");
    return;
  }
  map<double, map<string,double>> data;
  double timeStamp;
  //int count;
  countdown();
  Brain.resetTimer();
  driverControl = true;
  while (Brain.Timer.time(timeUnits::sec) < 15){
    timeStamp = Brain.Timer.time(timeUnits::msec);
    for (int i = 0; i < (sizeof(NAMES)/sizeof(NAMES[0])); i++){
      data[timeStamp][NAMES[i]] = USE[i].velocity(velocityUnits::pct);
    }
  }
  timeStamp = Brain.Timer.time(timeUnits::msec);
  for (int i = 0; i < (sizeof(NAMES)/sizeof(NAMES[0])); i++){
    data[timeStamp][NAMES[i]] = 0;
  }
  H.Screen.clearScreen();
  H.Screen.setCursor(1,1);
  H.Screen.print("Times Up!");
  H.Screen.setCursor(2,1);
  H.Screen.print("Saving Data...");
  H.rumble(rumbleLong);
  //Brain.SDcard.savefile("Recording.bin", &data, sizeof(data))
  ofstream outfile("output.bin", std::ios::binary);
  if (outfile.is_open()) {
    for (auto const& pair : data) {
      outfile.write(reinterpret_cast<const char*>(&pair), sizeof(pair));
    }
    outfile.close();
  }
  H.Screen.setCursor(2,1);
  H.Screen.clearLine(2);
  H.Screen.print("Data Saved");
}

void load(){
  //map<double, map<string,double>> readMap;
  ifstream infile("output.bin", std::ios::binary);
  if (infile.is_open()) {
    while (!infile.eof()) {
      pair<double, map<string,double>> pair;
      infile.read(reinterpret_cast<char*>(&pair), sizeof(pair));
      readMap.insert(pair);
    }
    infile.close();
  }
}

void play(){
  Brain.resetTimer();
  while(true){
    auto it = readMap.lower_bound(Brain.Timer.time(timeUnits::msec));
    for ( const auto &myPair : it ) {
      //
    }      
  }
}
