/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\T0825640Z                                        */
/*    Created:      Thu Jan 12 2023                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

motor motor1 = motor(PORT1, gearSetting::ratio18_1,false);
motor motor2 = motor(PORT2, gearSetting::ratio18_1,false);
motor motor3 = motor(PORT3, gearSetting::ratio18_1,false);
motor motor4 = motor(PORT4, gearSetting::ratio18_1,false);
motor motor5 = motor(PORT5, gearSetting::ratio18_1,false);
motor motor6 = motor(PORT6, gearSetting::ratio18_1,false);
motor motor7 = motor(PORT7, gearSetting::ratio18_1,false);
motor motor8 = motor(PORT8, gearSetting::ratio18_1,false);
motor motor9 = motor(PORT9, gearSetting::ratio18_1,false);
motor motor10 = motor(PORT10, gearSetting::ratio18_1,false);
motor motor11 = motor(PORT11, gearSetting::ratio18_1,false);
motor motor12 = motor(PORT12, gearSetting::ratio18_1,false);
motor motor13 = motor(PORT13, gearSetting::ratio18_1,false);
motor motor14 = motor(PORT14, gearSetting::ratio18_1,false);
motor motor15 = motor(PORT15, gearSetting::ratio18_1,false);
motor motor16 = motor(PORT16, gearSetting::ratio18_1,false);
motor motor17 = motor(PORT17, gearSetting::ratio18_1,false);
motor motor18 = motor(PORT18, gearSetting::ratio18_1,false);
motor motor19 = motor(PORT19, gearSetting::ratio18_1,false);
motor motor20 = motor(PORT20, gearSetting::ratio18_1,false);
motor motor21 = motor(PORT21, gearSetting::ratio18_1,false);

motor motorGrp[21] = {motor1,motor2,motor3,motor4,motor5,motor6,motor7,motor8,motor9,motor10,motor11,motor12,motor13,motor14,motor15,motor16,motor17,motor18,motor19,motor20,motor21};

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  while (1){
    for (motor test: motorGrp){
      test.spin(directionType::fwd);
    }
    task::sleep(15000);
    for (motor test: motorGrp){
      test.spin(directionType::rev);
    }
    task::sleep(15000);
    for (motor test: motorGrp){
      test.stop();
    }
    task::sleep(15000);
  }
}
