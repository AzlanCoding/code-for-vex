/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\azlan                                            */
/*    Created:      Fri Dec 30 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

motor TestMotor = motor(PORT1, gearSetting::ratio18_1, false);

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Brain.Screen.clearScreen();
  TestMotor.setVelocity(50,velocityUnits::pct);
  while (true){
    TestMotor.spin(directionType::fwd);
    Brain.Screen.setCursor(1,1);
    Brain.Screen.clearLine();
    Brain.Screen.print("Motor efficientcy: ");
    Brain.Screen.setCursor(1,20);
    Brain.Screen.print(TestMotor.efficiency(percentUnits::pct));
    Brain.Screen.setCursor(1,25);
    Brain.Screen.print("%%");
    task::sleep(200);
  }
}
