/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       AzlanCoding                                               */
/*    Created:      Thu Mar 23 2023                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// expand               digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;
// define your global instances of motors and other devices here

float highS = 12;
float lowS = 11;

motor LF = motor(PORT6, gearSetting::ratio18_1, true);
motor LB = motor(PORT8, gearSetting::ratio18_1, false);
motor RF = motor(PORT9, gearSetting::ratio18_1, false);
motor RB = motor(PORT7, gearSetting::ratio18_1, true);

motor intake = motor(PORT5, gearSetting::ratio18_1, true);
motor indexer = motor(PORT1, gearSetting::ratio18_1, false);

motor PriFly = motor(PORT11, gearSetting::ratio6_1, true);
motor SecFly = motor(PORT16, gearSetting::ratio6_1, false);

//digital_out expand = digital_out(Brain.ThreeWirePort.A);

controller H = controller(primary);
controller V = controller(partner);

motor base[4] = {LF,RF,LB,RB};
motor left[2] = {LF,LB};
motor right[2] = {RF,RB};

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
  expand.set(false);

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
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    LF.spin(directionType::fwd, H.Axis3.value()/10.583, voltageUnits::volt);
    LB.spin(directionType::fwd, H.Axis3.value()/10.583, voltageUnits::volt);
    RF.spin(directionType::fwd, H.Axis2.value()/10.583, voltageUnits::volt);
    RB.spin(directionType::fwd, H.Axis2.value()/10.583, voltageUnits::volt);

    intake.spin(directionType::rev, V.Axis2.value()/10.583, voltageUnits::volt);
    if (V.Axis2.value() < 0){
      indexer.spin(directionType::rev, V.Axis2.value()/10.583, voltageUnits::volt);
    }
    else if (V.ButtonL1.pressing()){
      indexer.spin(directionType::fwd, 12, voltageUnits::volt);
      intake.spin(directionType::rev, 12, voltageUnits::volt);
    }
    else if (H.ButtonR1.pressing()){
      indexer.spin(directionType::rev, 12, voltageUnits::volt);
      intake.spin(directionType::fwd, 12, voltageUnits::volt);
    }
    else{
      indexer.spin(directionType::rev, 0, voltageUnits::volt);
    }

    if (V.ButtonR2.pressing()){
      PriFly.spin(directionType::fwd, highS, voltageUnits::volt);
      SecFly.spin(directionType::fwd, highS, voltageUnits::volt);
    }
    else if (V.ButtonR1.pressing()){
      PriFly.spin(directionType::fwd, lowS, voltageUnits::volt);
      SecFly.spin(directionType::fwd, lowS, voltageUnits::volt);
    }
    else{
      PriFly.spin(directionType::fwd, 0, voltageUnits::volt);
      SecFly.spin(directionType::fwd, 0, voltageUnits::volt);
    }

    if (H.ButtonX.pressing()){
      expand.set(true);
      //task::sleep(500);
    }
    else{
      expand.set(false);
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
