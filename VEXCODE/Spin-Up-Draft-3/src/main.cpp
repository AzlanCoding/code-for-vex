/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       AzlanCoding                                               */
/*    Created:      Tue Sep 20 2022                                           */
/*    Description:  Spin Up Draft 3                                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Drivetrain           drivetrain    1, 10, D        
// ClawMotor            motor         3               
// ArmMotor             motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
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

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
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
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  Console.init();
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
