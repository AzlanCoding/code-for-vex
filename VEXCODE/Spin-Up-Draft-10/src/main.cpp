/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       AzlanCoding                                               */
/*    Created:      Tue Jan 31 2023                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Piston               digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
/*-------------------
|   Motor Legend    |
|                   |
|   LFF---v---RFF   |
|   LFB---|---RFB   |
|   LBF---|---RBF   |
|   LBB---^---RBB   |
|                   |-------------------*/
motor LFF = motor(PORT12, gearSetting::ratio18_1, false);
motor LFB = motor(PORT11, gearSetting::ratio18_1, true);
motor LBF = motor(PORT17, gearSetting::ratio18_1, true);
motor LBB = motor(PORT18, gearSetting::ratio18_1, false);
motor RFF = motor(PORT1, gearSetting::ratio18_1, true);
motor RFB = motor(PORT3, gearSetting::ratio18_1, false);
motor RBF = motor(PORT6, gearSetting::ratio18_1, false);
motor RBB = motor(PORT7, gearSetting::ratio18_1, true);

//digital_out Piston = digital_out(Brain.ThreeWirePort.A);

controller H = controller(primary);


motor Left[4] = {LFF,LFB,LBF,LBB};
motor Right[4] = {RFF,RFB,RBF,RBB};



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
  Piston.set(false);

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
    for (motor left: Left){
      left.spin(directionType::fwd,H.Axis3.value()/11.5,voltageUnits::volt);
    }

    for (motor right: Right){
      right.spin(directionType::fwd,H.Axis2.value()/11.5,voltageUnits::volt);
    }

    if (H.ButtonL2.pressing()==1){
      Piston.set(true);
    }

    if (H.ButtonL1.pressing()==1){
      Piston.set(false);
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
