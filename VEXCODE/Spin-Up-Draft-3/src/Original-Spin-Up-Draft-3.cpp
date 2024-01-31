#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.
#pragma endregion VEXcode Generated Robot Configuration

// ----------------------------------------------------------------------------
//                                                                            
//    Project:       Spin Up Draft 3
//    Author:        AzlanCoding
//    Created:       6 September 2022 (Tuesday) 08:52hrs
//    Configuration: None
//                                                                            
// ----------------------------------------------------------------------------

// Include the V5 Library
#include "vex.h"
// And other libraries
#include <string>
#include <cstring>
#include "vex_console.h"
#include <iostream>

// Allows for easier use of the VEX Library
using namespace vex;

// Begin project code

// Start Init
controller H = controller(primary);
controller V = controller(partner);

vex::motor LF = vex::motor(PORT1, gearSetting::ratio18_1, false);
vex::motor LB = vex::motor(PORT2, gearSetting::ratio18_1, false);
vex::motor RF = vex::motor(PORT3, gearSetting::ratio18_1, true);
vex::motor RB = vex::motor(PORT4, gearSetting::ratio18_1, true);

vex::motor Intake = vex::motor(PORT20, gearSetting::ratio18_1, false);

vex::console     Console;

int maxfilter(int input){
  if (input >= 100){
    return 100;
  } else if (input <= -100){
    return -100;
  } else{
    return input;
  }
}

int speed = 0;
int oldspeed;
std::string olddir;
std::string direction;
void diac(int a1, int a2, int a3, int a4, bool 上, bool 下, bool 左, bool 右){
  if (a1 <= -32){
    direction = "↶Spining Left";
    //H.Screen.print("↶Spining Left");
    //Brain.Screen.print("↶Spining Left");
    speed = maxfilter(a1);
    RF.spin(directionType::fwd, speed, velocityUnits::pct);
    LF.spin(directionType::rev, speed, velocityUnits::pct);
    RB.spin(directionType::fwd, speed, velocityUnits::pct);
    LB.spin(directionType::rev, speed, velocityUnits::pct);
  } else if(a1 >= 32){
    direction = "↷Spinning Right";
    //H.Screen.print("↷Spinning Right");
    //Brain.Screen.print("↷Spinning Right");
    speed = maxfilter(a1);
    RF.spin(directionType::rev, speed, velocityUnits::pct);
    LF.spin(directionType::fwd, speed, velocityUnits::pct);
    RB.spin(directionType::rev, speed, velocityUnits::pct);
    LB.spin(directionType::fwd, speed, velocityUnits::pct);
  } else if (上){
    direction = "⤻PivotFront";
    //H.Screen.print("⤻PivotFront");
    //Brain.Screen.print("⤻PivotFront");
    if (a2 >= 10 || a2 <= -10){
      speed = maxfilter(a2);
    } else {
      speed = 0;
    }
    RF.setStopping(brakeType::brake);
    LF.setStopping(brakeType::brake);
    RB.setStopping(brakeType::coast);
    LB.setStopping(brakeType::coast);


    RF.stop();
    LF.stop();
    RB.spin(directionType::fwd, speed, velocityUnits::pct);
    LB.spin(directionType::rev, speed, velocityUnits::pct);
  } else if (下){
    direction = "↷PivotBack";
    //H.Screen.print("↷PivotBack");
    //Brain.Screen.print("↷PivotBack");
    if (a2 >= 10 || a2 <= -10){
      speed = maxfilter(a2);
    } else {
      speed = 0;
    }
    RF.setStopping(brakeType::coast);
    LF.setStopping(brakeType::coast);
    RB.setStopping(brakeType::brake);
    LB.setStopping(brakeType::brake);


    RF.spin(directionType::rev, speed, velocityUnits::pct);
    LF.spin(directionType::fwd, speed, velocityUnits::pct);
    RB.stop();
    LB.stop(); 
  } else if (a3 >= 32 && a4 >= 32){
    direction = "↖FR";
    //H.Screen.print("↖FR");
    //Brain.Screen.print("↖FR");
    if (a2 >= 10 || a2 <= -10){
      speed = maxfilter(a2);
    } else {
      speed = 0;
    }
    RF.setStopping(brakeType::coast);
    LF.setStopping(brakeType::brake);
    RB.setStopping(brakeType::brake);
    LB.setStopping(brakeType::coast);


    RF.spin(directionType::fwd, speed, velocityUnits::pct);
    LF.stop();
    RB.stop();
    LB.spin(directionType::fwd, speed, velocityUnits::pct);
  } else if (a3 <= -32 && a4 <= -32){
    direction = "↙BL";
    //H.Screen.print("↙BL");
    //Brain.Screen.print("↙BL");
    if (a2 >= 10 || a2 <= -10){
      speed = maxfilter(a2);
    } else {
      speed = 0;
    }
    RF.setStopping(brakeType::brake);
    LF.setStopping(brakeType::coast);
    RB.setStopping(brakeType::coast);
    LB.setStopping(brakeType::brake);

    RF.stop();
    LF.spin(directionType::rev, speed, velocityUnits::pct);
    RB.spin(directionType::rev, speed, velocityUnits::pct);
    LB.stop();
  } else if (a3 >= 32 && a4 <= -32){
    direction = "↗FL";
    //H.Screen.print("↗FL");
    //Brain.Screen.print("↗FL");
    if (a2 >= 10 || a2 <= -10){
      speed = maxfilter(a2);
    } else {
      speed = 0;
    }
    RF.setStopping(brakeType::coast);
    LF.setStopping(brakeType::brake);
    RB.setStopping(brakeType::brake);
    LB.setStopping(brakeType::coast);
    
    RF.spin(directionType::fwd, speed, velocityUnits::pct);
    LF.stop();
    RB.stop();
    LB.spin(directionType::fwd, speed, velocityUnits::pct);
  } else if (a3 <= -32 && a4 >= 32){
    direction = "↙BR";
    //H.Screen.print("↙BR");
    //Brain.Screen.print("↙BR");
    if (a2 >= 10 || a2 <= -10){
      speed = maxfilter(a2);
    } else {
      speed = 0;
    }
    RF.setStopping(brakeType::coast);
    LF.setStopping(brakeType::brake);
    RB.setStopping(brakeType::brake);
    LB.setStopping(brakeType::coast);

    RF.stop();
    LF.spin(directionType::rev, speed, velocityUnits::pct);
    RB.spin(directionType::rev, speed, velocityUnits::pct);
    LB.stop();
  } else if (a3 < 0 && a4 >= -32 && a4 <= 32){
    direction = "↓Rvs";
    //H.Screen.print("↓Rvs");
    //Brain.Screen.print("↓Rvs");
    if (a2 >= 10 || a2 <= -10){
      speed = maxfilter(a2);
    } else {
      speed = 0;
    }
    RF.setStopping(brakeType::coast);
    LF.setStopping(brakeType::coast);
    RB.setStopping(brakeType::coast);
    LB.setStopping(brakeType::coast);

    RF.spin(directionType::rev, speed, velocityUnits::pct);
    LF.spin(directionType::rev, speed, velocityUnits::pct);
    RB.spin(directionType::rev, speed, velocityUnits::pct);
    LB.spin(directionType::rev, speed, velocityUnits::pct);
  } else if (a3 > 0 && a4 >= -32 && a4 <= 32){
    direction = "↑Fwd";
    //H.Screen.print("↑Fwd");
    //Brain.Screen.print("↑Fwd");
    if (a2 >= 10 || a2 <= -10){
      speed = maxfilter(a2);
    } else {
      speed = 0;
    }
    RF.setStopping(brakeType::coast);
    LF.setStopping(brakeType::coast);
    RB.setStopping(brakeType::coast);
    LB.setStopping(brakeType::coast);

    RF.spin(directionType::fwd, speed, velocityUnits::pct);
    LF.spin(directionType::fwd, speed, velocityUnits::pct);
    RB.spin(directionType::fwd, speed, velocityUnits::pct);
    LB.spin(directionType::fwd, speed, velocityUnits::pct);
  } else if (a4 < 0 && a3 >= -32 && a3 <= 32){
    direction = "←Left";
    //H.Screen.print("←Left");
    //Brain.Screen.print("←Left");
    if (a2 >= 10 || a2 <= -10){
      speed = maxfilter(a2);
    } else {
      speed = 0;
    }
    RF.setStopping(brakeType::coast);
    LF.setStopping(brakeType::coast);
    RB.setStopping(brakeType::coast);
    LB.setStopping(brakeType::coast);

    RF.spin(directionType::fwd, speed, velocityUnits::pct);
    LF.spin(directionType::rev, speed, velocityUnits::pct);
    RB.spin(directionType::rev, speed, velocityUnits::pct);
    LB.spin(directionType::fwd, speed, velocityUnits::pct);
  } else if (a4 > 0 && a3 >= -32 && a3 <= 32){
    direction = "→Right";
    //H.Screen.print("→Right");
    //Brain.Screen.print("→Right");
    if (a2 >= 10 || a2 <= -10){
      speed = maxfilter(a2);
    } else {
      speed = 0;
    }
    RF.spin(directionType::rev, speed, velocityUnits::pct);
    LF.spin(directionType::fwd, speed, velocityUnits::pct);
    RB.spin(directionType::fwd, speed, velocityUnits::pct);
    LB.spin(directionType::rev, speed, velocityUnits::pct);
  } else {
    direction = "Idle";
    //H.Screen.print("Idle");
    //Brain.Screen.print("Idle");
    speed = 0;
    
    RF.setStopping(brakeType::coast);
    LF.setStopping(brakeType::coast);
    RB.setStopping(brakeType::coast);
    LB.setStopping(brakeType::coast);

    RF.spin(directionType::rev, speed, velocityUnits::pct);
    LF.spin(directionType::fwd, speed, velocityUnits::pct);
    RB.spin(directionType::fwd, speed, velocityUnits::pct);
    LB.spin(directionType::rev, speed, velocityUnits::pct);
  }

}








void stop(){
  LF.spin(directionType::fwd, 0, velocityUnits::pct);
  LB.spin(directionType::fwd, 0, velocityUnits::pct);
  RF.spin(directionType::fwd, 0, velocityUnits::pct);
  RB.spin(directionType::fwd, 0, velocityUnits::pct);
}

void None(){
  LF.setStopping(brakeType::coast);
  LB.setStopping(brakeType::coast);
  RF.setStopping(brakeType::coast);
  RB.setStopping(brakeType::coast);
}

void checkB(){
  if (H.ButtonB.pressing() || H.ButtonR1.pressing() || H.ButtonR2.pressing())
    {
      H.Screen.setCursor(3,1);
      H.Screen.clearLine(3);
      H.Screen.print("Stopping...");
      stop();
      LF.setStopping(brakeType::brake);
      LB.setStopping(brakeType::brake);
      RF.setStopping(brakeType::brake);
      RB.setStopping(brakeType::brake);
      task::sleep(500);
      LF.setStopping(brakeType::coast);
      LB.setStopping(brakeType::coast);
      RF.setStopping(brakeType::coast);
      RB.setStopping(brakeType::coast);
      H.Screen.setCursor(3,1);
      H.Screen.clearLine();
      H.Screen.print("");
      H.Screen.clearScreen();
      H.Screen.setCursor(1,1);
      H.Screen.clearLine();
      H.Screen.print("Direction:");
      H.Screen.print(direction.c_str());
      H.Screen.setCursor(2,1);
      H.Screen.clearLine();
      H.Screen.print("Speed:");
      H.Screen.print(speed);
    }
}

void preAutonomous(void) {
  // actions to do when the program starts
  Brain.Screen.clearScreen();
  Brain.Screen.print("pre auton code");
  wait(1, seconds);
}

void autonomous(void) {
  Brain.Screen.clearScreen();
  Brain.Screen.print("autonomous code");
  // place automonous code here
}

void userControl(void) {
  H.Screen.clearScreen();
  H.Screen.clearScreen();
  //LF.setStopping(brakeType::coast);
  //LB.setStopping(brakeType::coast);
  //RF.setStopping(brakeType::coast);
  //RB.setStopping(brakeType::coast);
  //int speed = 50;
  //H.Screen.setCursor(1,1);
  //H.Screen.clearLine();
  //H.Screen.print("speed:");
  //H.Screen.print(speed);
  // place driver control in this while loop
  while (true) {
    if (H.ButtonX.pressing())
    {
      H.Screen.setCursor(2,1);
      H.Screen.clearLine();
      H.Screen.print("Exiting...");
      vexSystemExitRequest();
      wait(1, seconds);
    }

    if (H.ButtonB.pressing() || H.ButtonR1.pressing() || H.ButtonR2.pressing())
    {
      H.Screen.setCursor(3,1);
      H.Screen.clearLine(3);
      H.Screen.print("Stopping...");
      stop();
      LF.setStopping(brakeType::brake);
      LB.setStopping(brakeType::brake);
      RF.setStopping(brakeType::brake);
      RB.setStopping(brakeType::brake);
      wait(0.5, seconds);
      LF.setStopping(brakeType::coast);
      LB.setStopping(brakeType::coast);
      RF.setStopping(brakeType::coast);
      RB.setStopping(brakeType::coast);
      H.Screen.setCursor(3,1);
      H.Screen.clearLine();
      H.Screen.print("");
      H.Screen.clearScreen();
      H.Screen.setCursor(1,1);
      H.Screen.clearLine(1);
      H.Screen.print("Direction:");
      H.Screen.print(direction.c_str());
      H.Screen.setCursor(2,1);
      H.Screen.clearLine(2);
      H.Screen.print("Speed:");
      H.Screen.print(speed);
    }
    /*
    RF.spin(directionType::fwd, fwd - sid - tun, velocityUnits::pct);
    LF.spin(directionType::fwd, fwd + sid + tun, velocityUnits::pct);
    RB.spin(directionType::fwd, fwd + sid - tun, velocityUnits::pct);
    LB.spin(directionType::fwd, fwd - sid + tun, velocityUnits::pct);*/

    Intake.spin(vex::directionType::fwd, V.Axis3.value(), velocityUnits::pct);
    if (Intake.torque(Nm) >= 1.5){
      V.rumble(rumbleLong);
    }
    /*Brain.Screen.setCursor(1, 1);
    Brain.Screen.clearLine();
    Brain.Screen.print("Axis1 =");
    Brain.Screen.print(H.Axis1.value());
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.clearLine();
    Brain.Screen.print("Axis2 =");
    Brain.Screen.print(H.Axis2.value());
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.clearLine();
    Brain.Screen.print("Axis3 =");
    Brain.Screen.print(H.Axis3.value());
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.clearLine();
    Brain.Screen.print("Axis4 =");
    Brain.Screen.print(H.Axis4.value()); Debug*/
    diac(H.Axis1.value(),H.Axis2.value(),H.Axis3.value(),H.Axis4.value(),H.ButtonUp.pressing(),H.ButtonDown.pressing(),H.ButtonLeft.pressing(),H.ButtonRight.pressing());
    if (olddir != direction) {
      //Brain.Screen.setCursor(5, 1); Debug
      H.Screen.setCursor(1, 1);
      H.Screen.clearLine();
      H.Screen.print("Direction: %s",direction.c_str());
      /*Brain.Screen.setCursor(1, 1);
      Brain.Screen.clearLine();
      Brain.Screen.print("Direction: %s",direction.c_str()); Use console*/
      std::cout << "Direction: " << direction.c_str() << std::endl;
      olddir = direction;
    }
    if (oldspeed != speed){
      //Brain.Screen.setCursor(6, 1); Debug
      /*Brain.Screen.setCursor(1, 1);
      Brain.Screen.clearLine(); Use console*/
      Brain.Screen.print("Speed: %d",speed);
      std::cout << "Speed" << speed << std::endl;
      H.Screen.setCursor(2, 1);
      H.Screen.clearLine(2);
      H.Screen.print("Speed: %d",speed);
      //H.Screen.print(speed);
      //Brain.Screen.print(speed);
      oldspeed = speed;
    }
  }
}

int main() {
  Console.init();
  // create competition instance
  competition Competition;

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  // Run the pre-autonomous function.
  preAutonomous();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}