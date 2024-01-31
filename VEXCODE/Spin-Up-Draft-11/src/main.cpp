/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       AzlanCoding                                               */
/*    Created:      Thu Jan 19 2023                                           */
/*    Description:                                                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// limitSwitch          limit         B               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <iostream>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

controller H = controller(primary);
controller V = controller(partner);

motor LF = motor(PORT3, gearSetting::ratio18_1, true);
motor LM = motor(PORT2, gearSetting::ratio18_1, true);
motor LB = motor(PORT1, gearSetting::ratio18_1, false);
motor RF = motor(PORT10, gearSetting::ratio18_1, false);
motor RM = motor(PORT9, gearSetting::ratio18_1, false);
motor RB = motor(PORT8, gearSetting::ratio18_1, true);

motor intake = motor(PORT20, gearSetting::ratio18_1, true);//Fwd intakes

motor cata = motor(PORT11, gearSetting::ratio36_1, true);

//digital_in limitSwitch = digital_in(Brain.ThreeWirePort.A);


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
  Brain.setTimer(105,timeUnits::sec);
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    LF.spin(directionType::fwd, H.Axis3.value()/11.5, voltageUnits::volt);
    LF.spin(directionType::fwd, H.Axis3.value()/11.5, voltageUnits::volt);
    LB.spin(directionType::fwd, H.Axis3.value()/11.5, voltageUnits::volt);
    RF.spin(directionType::fwd, H.Axis2.value()/11.5, voltageUnits::volt);
    RF.spin(directionType::fwd, H.Axis2.value()/11.5, voltageUnits::volt);
    RB.spin(directionType::fwd, H.Axis2.value()/11.5, voltageUnits::volt);

    intake.spin(directionType::fwd, V.Axis2.value()/11.5, voltageUnits::volt);

    if (V.ButtonR1.pressing()==1 /*&& cata.isSpinning()==0 */&& limitSwitch.value()==0){
      cata.spin(directionType::fwd, 11.5, voltageUnits::volt);
    } 
    else if ((H.ButtonR1.pressing()==1) /*&& cata.isSpinning()==0*/){
      cata.spin(directionType::fwd, 5, voltageUnits::volt);
    } 
    else {
      //cata.stop(brakeType::hold);
      cata.spin(directionType::fwd, 0, voltageUnits::volt);
    }

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
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
