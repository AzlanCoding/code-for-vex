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

using namespace vex;

motor LF = motor(PORT17, gearSetting::ratio18_1, false);
motor LB = motor(PORT18, gearSetting::ratio18_1, false);
motor RF = motor(PORT19, gearSetting::ratio18_1, true);
motor RB = motor(PORT20, gearSetting::ratio18_1, true);

motor ArmL = motor(PORT9, gearSetting::ratio36_1, false);
motor ArmR = motor(PORT10, gearSetting::ratio36_1, true);

controller H = controller(primary);

motor motors[4] = {LF,RF,RB,LB};

bool braking = false;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  H.Screen.clearScreen();
  H.Screen.setCursor(1,1);
  H.Screen.print("BRAKE: OFF");

  while (true){
    if (H.ButtonB.pressing()){
      if (braking == false){
        for (motor m: motors){
          m.stop(brakeType::hold);
        }
        H.Screen.clearScreen();
        H.Screen.setCursor(1,1);
        H.Screen.print("BRAKE: ON");
        braking = true;
        //H.rumble(rumbleLong);
        //while(H.ButtonB.pressing()){}
      }
      else{
        for (motor m: motors){
          m.stop(brakeType::coast);
        }
        H.Screen.clearScreen();
        H.Screen.setCursor(1,1);
        H.Screen.print("BRAKE: OFF");
        braking = false;
        //H.rumble(rumbleLong);
      }
      while(H.ButtonB.pressing()){}
    }
    else if (braking == false){
      LF.spin(directionType::fwd, H.Axis3.value()/10.583, voltageUnits::volt);
      LB.spin(directionType::fwd, H.Axis3.value()/10.583, voltageUnits::volt);
      RF.spin(directionType::fwd, H.Axis2.value()/10.583, voltageUnits::volt);
      RB.spin(directionType::fwd, H.Axis2.value()/10.583, voltageUnits::volt);
    }
    if (H.ButtonL1.pressing()){
      ArmL.spin(directionType::fwd, 12, voltageUnits::volt);
      ArmR.spin(directionType::fwd, 0, voltageUnits::volt);
    }
    else if (H.ButtonL2.pressing()){
      ArmL.spin(directionType::rev, 12, voltageUnits::volt);
      ArmR.spin(directionType::fwd, 0, voltageUnits::volt);
    }
    else if (H.ButtonR1.pressing()){
      ArmL.spin(directionType::fwd, 0, voltageUnits::volt);
      ArmR.spin(directionType::fwd, 12, voltageUnits::volt);
    }
    else if (H.ButtonR2.pressing()){
      ArmL.spin(directionType::rev, 0, voltageUnits::volt);
      ArmR.spin(directionType::rev, 12, voltageUnits::volt);
    }
    else{
      ArmL.spin(directionType::fwd, 0, voltageUnits::volt);
      ArmR.spin(directionType::fwd, 0, voltageUnits::volt);
    }
  }
  
}
