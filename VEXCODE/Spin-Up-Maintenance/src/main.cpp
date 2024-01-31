/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\T0825640Z                                        */
/*    Created:      Fri Nov 04 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

vex::controller H = vex::controller(primary);
motor m1 = motor(PORT12, gearSetting::ratio18_1,false);
motor m2 = motor(PORT20, gearSetting::ratio18_1,false);

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  while(true){
    H.Screen.setCursor(1,1);
    H.Screen.clearLine();
    H.Screen.print(m1.efficiency(vex::percentUnits::pct));
    H.Screen.setCursor(2,1);
    H.Screen.clearLine();
    H.Screen.print(m2.efficiency(vex::percentUnits::pct));
  }
  
}
