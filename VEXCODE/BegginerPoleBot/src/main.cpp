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

motor L = motor(PORT1,gearSetting::ratio18_1,false);
motor R = motor(PORT10,gearSetting::ratio18_1,true);

controller H = controller(primary);

double filter(int control){
  int maxSpeed = 70;
  if (control >= 100){
    return maxSpeed;
  } else if (control <= -100){
    return maxSpeed*-1;
  }else{
    double x = control /10;
    return maxSpeed*(x/10);
  }
}


int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  //Brain.Screen.print("Hello World!");
  //H.Screen.clearScreen();
  while(true){
    /*H.Screen.setCursor(1,1);
    H.Screen.clearLine();
    H.Screen.print(filter(H.Axis2.value()));
    H.Screen.setCursor(2,1);
    H.Screen.clearLine();
    H.Screen.print(H.Axis2.value());*/
    L.spin(directionType::fwd,filter(H.Axis3.value()),velocityUnits::pct);
    R.spin(directionType::fwd,filter(H.Axis2.value()),velocityUnits::pct);
  }

  
}
