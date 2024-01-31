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
#include <math.h>

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
|                   |
-------------------*/
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

//Global Variables
double wheelCircumference = 4 * 2.54 * M_PI; //(( wheel(inch) * 2.54)[convert to cm] * pi)The circumference of the wheel. (cm)
double wheelTrack = 34.2 * M_PI; //(distance between left and right wheels * pi)The distance a wheel will travel to spin the bot 360deg(cm)
bool slwDown = false;
bool turnin = false;
// these values are used to calculate turning for the bot

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

bool slowDown(double initial, double target){
  double diff = target - initial;
  if (target < 0){
    diff = diff * -1;
  }
  if (slwDown == false){
    Brain.Screen.setCursor(4,1);
    Brain.Screen.clearLine();
    Brain.Screen.print(target);
    Brain.Screen.print(" ");
    Brain.Screen.print(initial);
    Brain.Screen.print(" ");
    Brain.Screen.print(diff);
  }
  if (diff <= 0.15){
    if (slwDown == false){
      Brain.Screen.print(" ");
      Brain.Screen.print("trig at ");
      Brain.Screen.print(diff);
    }
    return true;
  }
  return false;
}

bool check(double LI, double RI, double LT, double RT){// RI stands for Right Initial while LT stands for Left Target
  //bool reverse[2] = {false,false};
  Brain.Screen.setCursor(1,1);
  Brain.Screen.clearLine();
  Brain.Screen.setCursor(1,1);
  Brain.Screen.print(LI);
  Brain.Screen.print(" ");
  Brain.Screen.print(LT);
  Brain.Screen.setCursor(2,1);
  Brain.Screen.clearLine();
  Brain.Screen.print(RI);
  Brain.Screen.print(" ");
  Brain.Screen.print(RT);
  if (LI == LT && RI == RT){
    return false;
  }
  if (LT < 0 && RT < 0 && LI <= LT && RI <= RT){
    return false;
  }
  if (LT < 0 && RT > 0 && LI <= LT && RI >= RT){
    return false;
  }
  if ((LT > 0 && RT < 0 )&&(LI >= LT || RI <= RT)){
    turnin = true;
    return false;
  }
  /*if (LT > 0 && RT < 0 && LI >= LT && RI <= RT){
    return false;
  }*/
  if (LT > 0 && RT > 0 && LI >= LT && RI >= RT){
    return false;
  }
  return true;
}

void calibrate(double LI, double RI, double LT, double RT){
  double Temp;
  while (LI >= LT && RT >= RI){
    check(LI,RI,LT,RT);
    Temp = 0;
    for (motor l: Left){
      Temp += l.rotation(rotationUnits::rev);
    }
    LI = Temp/4;
    Temp = 0;
    for (motor r: Right){
      Temp += r.rotation(rotationUnits::rev);
    }
    RI = Temp/4;
    if (LT > 0 && LI >= LT){
      for (motor l: Left){
        l.spin(directionType::rev,0,voltageUnits::volt);
      }
    }
    if (LT < 0 && LI <= LT){
      for (motor l: Left){
        l.spin(directionType::rev,0,voltageUnits::volt);
      }
    }
    if (RT > 0 && RI >= RT){
      for (motor r: Right){
        r.spin(directionType::rev,0,voltageUnits::volt);
      }
    }
    if (RT < 0 && RI <= RT){
      for (motor r: Right){
        r.spin(directionType::rev,0,voltageUnits::volt);
      }
    }
  }
}

int turnFix(double left, double right){
  if (left == right){
    return 100;
  }
  return 50;
}

void move(double left, double right){
  double lRev = 0;
  double rRev = 0;
  for (motor l: Left){
    l.resetRotation();
    if (left > 0){
      l.spin(directionType::fwd,turnFix(left,right),velocityUnits::pct);
    } else if (left < 0){
      l.spin(directionType::rev,turnFix(left,right),velocityUnits::pct);
    }
  }
  for (motor r: Right){
    r.resetRotation();
    if (right > 0){
      r.spin(directionType::fwd,turnFix(left,right),velocityUnits::pct);
    } else if (right < 0){
      r.spin(directionType::rev,turnFix(left,right),velocityUnits::pct);
    }
  }
  double Temp;
  while (check(lRev,rRev,left,right)){
    Temp = 0;
    for (motor l: Left){
      Temp += l.rotation(rotationUnits::rev);
    }
    lRev = Temp/4;
    Temp = 0;
    for (motor r: Right){
      Temp += r.rotation(rotationUnits::rev);
    }
    rRev = Temp/4;
    if ((slowDown(lRev,left) || slowDown(rRev,right))){
      slwDown = true;
      for (motor l: Left){
        if (left > 0){
          l.spin(directionType::fwd,1.5,voltageUnits::volt);
        } else if (left < 0){
          l.spin(directionType::rev,1.5,voltageUnits::volt);
        }
      }
      for (motor r: Right){
        if (right > 0){
          r.spin(directionType::fwd,1.5,voltageUnits::volt);
        } else if (right < 0){
          r.spin(directionType::rev,1.5,voltageUnits::volt);
        }
      }
    }
  }
  if (turnin == true){
    calibrate(lRev,rRev,left,right);
  }
  turnin = false;
  slwDown = false;
}


void stop(){
  for (motor l: Left){
    l.spin(directionType::rev,0,voltageUnits::volt);
  }
  for (motor r: Right){
    r.spin(directionType::rev,0,voltageUnits::volt);
  }
}

void rotate(double deg){
  double pct;
  pct = deg/(double)360;
  double rotations;
  rotations = wheelTrack / wheelCircumference * pct;
  move(rotations,rotations*-1);
  stop();
}

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
  Brain.Screen.print("Auton");
  //move(1,1);
  rotate(90);
  move(1,1);
  /*move(-1,-1);
  move(-0.5,0.5);
  move(1,-1);
  move(-0.5,0.5);
  //move(-1,-1);*/
  stop();
  Brain.Screen.print("Done");
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
  //Brain.Screen.print("HI");
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

    /*if (H.ButtonL1.pressing()==1){
      Piston.set(false);
    }*/
    
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
  //autonomous();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
