/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       AzlanCoding                                               */
/*    Created:      Thu Nov 03 2022                                           */
/*    Description:  Spin Up Competition                                       */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#define _USE_MATH_DEFINES

#include <math.h>
#include "vex.h"
#include "vex_console.h"
#include <iostream>
#include <string>
#include <cstring>
#include <sstream>
#include <thread>


using namespace vex;
using namespace std;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
vex::console     Console;

vex::controller H = vex::controller(primary);
vex::controller V = vex::controller(partner);

vex::motor LF = motor(PORT1,gearSetting::ratio18_1,false);      //Left Front
vex::motor LB = motor(PORT1,gearSetting::ratio18_1,false);      //Left Back

vex::motor RF = motor(PORT1,gearSetting::ratio18_1,false);      //Right Front
vex::motor RB = motor(PORT1,gearSetting::ratio18_1,false);      //Right Back

vex::motor Intake1 = motor(PORT1, gearSetting::ratio18_1,true); //Intake front motor
vex::motor Intake2 = motor(PORT2,gearSetting::ratio18_1,false); //Intake middle motor

vex::motor Fly1 = motor(PORT1,gearSetting::ratio18_1,false);    //Flywheel motor 2
vex::motor Fly2 = motor(PORT1,gearSetting::ratio18_1,false);    //Flywheel motor 1

digital_out Piston = digital_out(Brain.ThreeWirePort.A);

vex::controller controllers[2] = {H,V};

vex::motor Motors[8] = {LF,LB,RF,RB,Intake1,Intake2,Fly1,Fly2};
vex::motor Base[4] = {LF,LB,RF,RB};

//Global Variables
double wheelCircumference = 4 * 2.54 * M_PI; //(( wheel(inch) * 2.54)[convert to cm] * pi)The circumference of the wheel. (cm)
double wheelTrack = 31.5 * M_PI; //(distance between left and right wheels * pi)The distance a wheel will travel to spin the bot 360deg(cm)
// these values are used to calculate turning for the bot
double lowVolt = 8.1;   //lowspeed for flywheel
double highVolt = 10.9; //highspeed for flywheel
bool Auton_now = false; //Enable this to run the autonomous code when user-control is started. This helps to test the autonoumous code.

//to_string function patch due to Mingui https://stackoverflow.com/questions/12975341/to-string-is-not-a-member-of-std-says-g-mingw
namespace std {
    template<typename T>
    std::string to_string(const T &n) {
        std::ostringstream s;
        s << n;
        return s.str();
    }
}

//User-Control Functions
string getName(int i){
  string names[8] = {"Left-Front Motor","Left-Back Motor","Right-Back Motor","Intake Motor 1","Intake Motor 2","FlyWheel Motor 1","FlyWheel Motor 2"};
  return names[i];
}

void log(string message){
  cout << message << "\n";
}

void alert(string message){
  log("WARNING: "+message);
  for (controller i: controllers){
    i.rumble("-");
    i.Screen.clearScreen();
    i.Screen.setCursor(1,1);
    i.Screen.print("WARNING:");
    i.Screen.setCursor(2,1);
    i.Screen.print(message.c_str()); //Controller.Screen.print only accept c_string() https://www.vexforum.com/t/brain-screen-print-std-string-not-working/101006
  }
}

void MotorCheck() {
  while(true){
    for (int i = 1; i <= 8; i++) {
      if (Motors[i].temperature(vex::percentUnits::pct)<=0){
        alert(getName(i) + " is disconnected!");
      }       if (Motors[i].temperature(vex::percentUnits::pct) >= 0.65){
        alert(getName(i) + " is overheating.");
      }
      if (Motors[i].torque(vex::torqueUnits::Nm) >= 1.8){
        alert(getName(i) + " is close to stalling/is stalling.");
      }
      if (Motors[i].efficiency(vex::percentUnits::pct)<=0.50){
        alert(getName(i) + " is inefficeint.");
      }
      vex::task::sleep(500);
    }
  }
}

//Autonomus Functions
void move(double data,rotationUnits unit){
  Base[0].rotateFor(data,unit,false);
  Base[1].rotateFor(data,unit,false);
  Base[2].rotateFor(data,unit,false);
  Base[3].rotateFor(data,unit,true);
}

void FWD(double tiles){
  log("INFO: Moving " + to_string(tiles) + "tiles.");
  double rev;
  rev = tiles/(double)2;
  move(rev,rotationUnits::rev);
}
void FWD(int tiles){
  log("INFO: Moving " + to_string(tiles) + "tiles.");
  double rev;
  rev = tiles/(double)2;
  move(rev,rotationUnits::rev);
}

void Rotate(int deg){
  log("INFO: ROTATING " + to_string(deg) + "degrees.");
  double pct;
  pct = deg/(double)360;
  double rotations;
  rotations = wheelTrack / wheelCircumference * pct;
  rotations /= wheelCircumference; // same as rotations = rotations / wheelCircumference
  Base[0].rotateFor(rotations,rotationUnits::rev,false);
  Base[1].rotateFor(rotations,rotationUnits::rev,false);
  Base[2].rotateFor(rotations*-1,rotationUnits::rev,false);
  Base[3].rotateFor(rotations*-1,rotationUnits::rev,true);
  return;
}

void Flywheel(bool state1,double volts){// 'state` may be used by vex
  if (state1){
    Motors[7].spin(directionType::fwd,volts,voltageUnits::volt);
    Motors[8].spin(directionType::fwd,volts,voltageUnits::volt);
  } else{
    Motors[7].stop();
    Motors[8].stop();
  }
}

void Roller(bool state1){
  log("INFO: Roller set to value " + to_string(state1));
  if (state1){
    Motors[6].spin(directionType::fwd);
  } else{
    Motors[6].stop();
  }
}

//Autonomus and User-Control Functions
void Launch(){
  Piston.set(true);
  vex::task::sleep(200);
  Piston.set(false);
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
  for(motor i: Motors){
    i.setStopping(brakeType::coast);
  }
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
  log("Autonomous Started");
  FWD(0.25);
  Rotate(90);
  FWD(1);
  Rotate(90);
  Roller(true);
  FWD(1.5);
  Roller(false);
  FWD(-1.5);
  Rotate(-90);
  Roller(true);
  FWD(1.5);
  Roller(false);
  FWD(-3);
  Flywheel(true, highVolt);
  Rotate(180);
  log("INFO: Launching...");
  Launch();
  FWD(0.5);
  log("INFO: Launching...");
  Launch();
  log("INFO: Waiting for flywheel...");
  vex::task::sleep(1000);
  Flywheel(true, lowVolt);
  log("INFO: Launching..");
  Launch();
  log("Autonomous Finished");
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
  if (Auton_now){
    pre_auton();
    autonomous();
    return;
  }
  for(motor i: Motors){
    i.setStopping(brakeType::coast);
  }
  thread Check(MotorCheck); //Background task to check motors.
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    LF.spin(directionType::fwd, H.Axis3.value(), velocityUnits::pct);
    LB.spin(directionType::fwd, H.Axis3.value(), velocityUnits::pct);
    RF.spin(directionType::fwd, H.Axis2.value(), velocityUnits::pct);
    RB.spin(directionType::fwd, H.Axis2.value(), velocityUnits::pct);

    Intake1.spin(directionType::fwd, V.Axis3.value(), velocityUnits::pct);
    Intake2.spin(directionType::fwd, V.Axis3.value(), velocityUnits::pct);

    if (V.ButtonR1.pressing() || V.ButtonR2.pressing()){
      Launch();
    }
    
    if (V.ButtonL1.pressing()){
      Motors[7].spin(directionType::fwd,lowVolt,voltageUnits::volt);
      Motors[8].spin(directionType::fwd,lowVolt,voltageUnits::volt);
    } else if (V.ButtonL2.pressing()){
      Motors[7].spin(directionType::fwd,highVolt,voltageUnits::volt);
      Motors[8].spin(directionType::fwd,highVolt,voltageUnits::volt);
    } else {
      Motors[7].stop();
      Motors[8].stop();
    }
    
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to prevent wasted resources.
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
