/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       AzlanCoding                                               */
/*    Created:      Mon Mar 29 2023                                           */
/*    Description:  Shaun VJC Competition Bot                                 */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
motor LF = motor(PORT10,gearSetting::ratio18_1,false);
motor LB = motor(PORT9,gearSetting::ratio18_1,false);
motor RF = motor(PORT20,gearSetting::ratio18_1,true);
motor RB = motor(PORT18,gearSetting::ratio18_1,true);

motor PriArmL = motor(PORT5,gearSetting::ratio36_1,false);
motor PriArmR = motor(PORT4,gearSetting::ratio36_1,true);

motor SecArmL = motor(PORT1,gearSetting::ratio18_1,false);
motor SecArmR = motor(PORT2,gearSetting::ratio18_1,true);

motor intake = motor(PORT15,gearSetting::ratio18_1,false);

controller H = controller(primary);
controller V = controller(partner);
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
  PriArmL.resetRotation();
  PriArmR.resetRotation();
  double speed = 6;
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    if (H.ButtonLeft.pressing()){
      RF.spin(directionType::fwd,speed,voltageUnits::volt);
      LF.spin(directionType::fwd,-speed,voltageUnits::volt);
      RB.spin(directionType::fwd,-speed,voltageUnits::volt);
      LB.spin(directionType::fwd,speed,voltageUnits::volt);
    }
    else if (H.ButtonRight.pressing()){
      RF.spin(directionType::fwd,-speed,voltageUnits::volt);
      LF.spin(directionType::fwd,speed,voltageUnits::volt);
      RB.spin(directionType::fwd,speed,voltageUnits::volt);
      LB.spin(directionType::fwd,-speed,voltageUnits::volt);
    }
    else if (H.ButtonUp.pressing()){
      RF.spin(directionType::fwd,speed,voltageUnits::volt);
      LF.spin(directionType::fwd,speed,voltageUnits::volt);
      RB.spin(directionType::fwd,speed,voltageUnits::volt);
      LB.spin(directionType::fwd,speed,voltageUnits::volt);
    }
    else if (H.ButtonDown.pressing()){
      RF.spin(directionType::fwd,-speed,voltageUnits::volt);
      LF.spin(directionType::fwd,-speed,voltageUnits::volt);
      RB.spin(directionType::fwd,-speed,voltageUnits::volt);
      LB.spin(directionType::fwd,-speed,voltageUnits::volt);
    }
    else{
    /*
    RF.setVelocity(double(H.Axis2.position()/(127/100) - (H.Axis1.position()/(127/100) + H.Axis4.position()/(127/100)/2)), percent);
    LF.setVelocity(double(H.Axis2.position()/(127/100) + (H.Axis1.position()/(127/100) + H.Axis4.position()/(127/100)/2)), percent);
    RB.setVelocity(double(H.Axis2.position()/(127/100) + (H.Axis1.position()/(127/100) - H.Axis4.position()/(127/100)/2)), percent);
    LB.setVelocity(double(H.Axis2.position()/(127/100) - (H.Axis1.position()/(127/100) - H.Axis4.position()/(127/100)/2)), percent);*/
    RF.spin(directionType::fwd, (double(H.Axis2.position()/(127/100) - (H.Axis1.position()/(127/100) + H.Axis4.position()/(127/100)/2))),percentUnits::pct);
    LF.spin(directionType::fwd, (double(H.Axis2.position()/(127/100) + (H.Axis1.position()/(127/100) + H.Axis4.position()/(127/100)/2))),percentUnits::pct);
    RB.spin(directionType::fwd, (double(H.Axis2.position()/(127/100) + (H.Axis1.position()/(127/100) - H.Axis4.position()/(127/100)/2))),percentUnits::pct);
    LB.spin(directionType::fwd, (double(H.Axis2.position()/(127/100) - (H.Axis1.position()/(127/100) - H.Axis4.position()/(127/100)/2))),percentUnits::pct);
    PriArmL.spin(directionType::fwd, V.Axis2.value()/10.583, voltageUnits::volt);
    PriArmR.spin(directionType::fwd, V.Axis2.value()/10.583, voltageUnits::volt);
    SecArmL.spin(directionType::fwd, V.Axis3.value()/10.583, voltageUnits::volt);
    SecArmR.spin(directionType::fwd, V.Axis3.value()/10.583, voltageUnits::volt);
    }
    V.Screen.clearScreen();
    V.Screen.setCursor(1,1);
    V.Screen.print(PriArmL.torque());
    /*if (PriArmL.torque()>= 1.5){
      PriArmL.stop(brakeType::coast);
      PriArmR.stop(brakeType::coast);
    }
    else if (V.Axis2.value() > 0){
      PriArmL.spin(directionType::fwd, V.Axis2.value()/10.583, voltageUnits::volt);
      PriArmR.spin(directionType::fwd, V.Axis2.value()/10.583, voltageUnits::volt);
    }
    else if (V.Axis2.value() < 0){
      PriArmL.spin(directionType::rev, 12, voltageUnits::volt);
      PriArmR.spin(directionType::rev, 12, voltageUnits::volt);
    }
    else{
      PriArmL.stop(brakeType::coast);
      PriArmR.stop(brakeType::coast);
    }*/

    if (V.ButtonR1.pressing()){
      intake.spin(directionType::fwd, 12, voltageUnits::volt);
    }
    else if (V.ButtonR2.pressing()){
      intake.spin(directionType::rev, 12, voltageUnits::volt);
    }
    else{
      intake.spin(directionType::fwd, 0, voltageUnits::volt);
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
