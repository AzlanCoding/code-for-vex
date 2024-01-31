/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\T0825640Z                                        */
/*    Created:      Thu Sep 22 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

motor rightmotor = motor(PORT2, gearSetting::ratio18_1,true);
motor leftmotor = motor(PORT1, gearSetting::ratio18_1,false);
controller control = controller();
void spins(int rig, int lef, int time){
rightmotor.spin(directionType::fwd, rig, velocityUnits::pct); 
leftmotor.spin(directionType::fwd, lef, velocityUnits::pct);
task::sleep(time);
rightmotor.stop();
leftmotor.stop();
}
int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  spins(30, 5, 10);
}
