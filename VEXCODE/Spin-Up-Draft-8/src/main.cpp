/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       AzlanCoding                                               */
/*    Created:      Fri Jan 13 2023                                           */
/*    Description:                                                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// DigitalOutA          digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <math.h>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
motor LeftFront = motor(PORT13,gearSetting::ratio18_1,false);
motor LeftBack = motor(PORT17,gearSetting::ratio18_1,false);
motor RightFront = motor(PORT16,gearSetting::ratio18_1,false);
motor RightBack = motor(PORT12,gearSetting::ratio18_1,false);

motor indexer = motor(PORT1,gearSetting::ratio18_1,false);
motor intake = motor(PORT7,gearSetting::ratio36_1,false);

motor SecondaryFly = motor(PORT6,gearSetting::ratio6_1,true);
motor PrimaryFly = motor(PORT5,gearSetting::ratio6_1,false);

//digital_out DigitalOutA  = digital_out(Brain.ThreeWirePort.A);

//motor Base[4] = {LeftFront,LeftBack,RightFront,RightBack};
//motor FlyWheels[2] = {SecondaryFly,PrimaryFly};

controller HController = controller(primary);
controller VController = controller(partner);

double highspeed = 11;
double lowspeed = 9;

double wheelCircumference = 4 * 2.54 * M_PI; //(( wheel(inch) * 2.54)[convert to cm] * pi)The circumference of the wheel. (cm)
double wheelTrack = 31.1 * M_PI; //(distance between left and right wheels * pi)The distance a wheel will travel to spin the bot 360deg. (cm)

int BotLocation = 1;

double posfilter(double val){
  if (val >= 0){
    return 0;
  }
  else{
    return val;
  }
}



void FwdMove(int speed){
  LeftFront.spin(directionType::fwd,speed,velocityUnits::pct);
  LeftBack.spin(directionType::fwd,speed,velocityUnits::pct);
  RightFront.spin(directionType::fwd,speed,velocityUnits::pct);
  RightBack.spin(directionType::fwd,speed,velocityUnits::pct);
}

void LMove(int speed){
  LeftFront.spin(directionType::fwd,speed,velocityUnits::pct);
  LeftBack.spin(directionType::fwd,speed,velocityUnits::pct);
  RightFront.spin(directionType::fwd,speed*-1,velocityUnits::pct);
  RightBack.spin(directionType::fwd,speed*-1,velocityUnits::pct);
}

void RMove(int speed){
  LeftFront.spin(directionType::fwd,speed*-1,velocityUnits::pct);
  LeftBack.spin(directionType::fwd,speed*-1,velocityUnits::pct);
  RightFront.spin(directionType::fwd,speed,velocityUnits::pct);
  RightBack.spin(directionType::fwd,speed,velocityUnits::pct);
}

bool checkComplete(bool array[], int size)
{
  for (int i = 0; i < size; i++)
  {
      if(array[i] == false)
      {
        return false;
      }
  }
  return true;
}

double Tiles(double spaces){//convert number of tiles to revolutions
  double rev;
  rev = spaces*60.96;
  rev /= wheelCircumference;
  return rev;
}


double Deg(int deg){//convert degrees bot needs to turn to revolutions
  double pct;
  pct = deg/(double)360;
  double rotations;
  rotations = wheelTrack / wheelCircumference * pct;
  return rotations;
}



void AutonMove(double left, double right){
  LeftFront.resetRotation();
  LeftBack.resetRotation();
  RightFront.resetRotation();
  RightBack.resetRotation();
  double rev[4] = {left,left,right,right};
  double LF;
  double LB;
  double RF;
  double RB;
  while(1){
    LF = LeftFront.rotation(rotationUnits::rev);
    LB = LeftBack.rotation(rotationUnits::rev);
    RF = RightFront.rotation(rotationUnits::rev);
    RB = RightBack.rotation(rotationUnits::rev);
    double encoderValues[4] = {LF,LB,RF,RB};
    motor Base[4] = {LeftFront,LeftBack,RightFront,RightBack};
    bool Completed[4] = {false,false,false,false};
    for (int i=0; i <= 4; i++){
      if (rev[i] > encoderValues[i]){
        Base[i].spin(directionType::fwd,100,velocityUnits::pct);
      }
      else if (rev[i] < encoderValues[i]){
        Base[i].spin(directionType::fwd,-100,velocityUnits::pct);
      }
      else{
        Base[i].spin(directionType::fwd,0,velocityUnits::pct);
        Completed[i] = true;
      }
    } 
    if (checkComplete(Completed,4) == true){
      for (motor i: Base){
        i.spin(directionType::fwd,0,voltageUnits::volt);
      }
      break;
    }

  }

}

void FWD(double spaces){
  AutonMove(Tiles(spaces),Tiles(spaces));
}


void Rotate(int deg){
  if (BotLocation == 2){
    AutonMove(Deg(deg)*-1,Deg(deg));
  }
  else {
    AutonMove(Deg(deg),Deg(deg)*-1);
  }
}

int FieldLocate(){
  return 3;
  while (1){
    bool changed = true;
    int selection = 1;
    if (changed == true){
      HController.Screen.clearScreen();
      HController.Screen.setCursor(1,3);
      HController.Screen.print("HighGoal on Left");
      HController.Screen.setCursor(2,3);
      HController.Screen.print("HighGoal on Right");
      HController.Screen.setCursor(3,3);
      HController.Screen.print("No Auton");
      HController.Screen.setCursor(selection,1);
      HController.Screen.print(">");
      changed = false;
    }
    if (HController.ButtonDown.pressing()==1&&selection!=3){
      selection++;
      changed = true;
    } 
    else if (HController.ButtonUp.pressing()==1&&selection!=1){
      selection--;
      changed = true;
    } 
    else if (HController.ButtonA.pressing()==1){
      return selection;
    } 
  }
}

//Autonomus Functions
/*void move(double data,rotationUnits unit){
  LeftFront.rotateFor(data,unit,100,velocityUnits::pct,false);
  LeftBack.rotateFor(data,unit,100,velocityUnits::pct,false);
  RightFront.rotateFor(data,unit,100,velocityUnits::pct,false);
  RightBack.rotateFor(data,unit,100,velocityUnits::pct,true);
}
*/
/*
void FWD(double tiles){
  double rev;
  rev = tiles*60.96;
  rev /= wheelCircumference;
  move(rev,rotationUnits::rev);
}
*/

/*void FWD(int tiles){
  double rev;
  rev = tiles*60.96;
  rev /= wheelCircumference;
  move(rev,rotationUnits::rev);
}
*/

void Fire(){
  PrimaryFly.spin(directionType::fwd,highspeed,voltageUnits::volt);
  SecondaryFly.spin(directionType::fwd,highspeed,voltageUnits::volt);
  vex::task::sleep(500);
  indexer.rotateFor(directionType::fwd,3,rotationUnits::rev,true);
  vex::task::sleep(500);
  indexer.rotateFor(directionType::fwd,3,rotationUnits::rev,true);
  PrimaryFly.spin(directionType::fwd,0,voltageUnits::volt);
  SecondaryFly.spin(directionType::fwd,0,voltageUnits::volt);
  return;
}

/*
void Rotate(int deg){
  double pct;
  pct = deg/(double)360;
  double rotations;
  rotations = wheelTrack / wheelCircumference * pct;
  switch (BotLocation){
    default:
      LeftFront.rotateFor(rotations,rotationUnits::rev,false);
      LeftBack.rotateFor(rotations,rotationUnits::rev,false);
      RightFront.rotateFor(rotations*-1,rotationUnits::rev,false);
      RightBack.rotateFor(rotations*-1,rotationUnits::rev,true);
    case 2:
      LeftFront.rotateFor(rotations*-1,rotationUnits::rev,false);
      LeftBack.rotateFor(rotations*-1,rotationUnits::rev,false);
      RightFront.rotateFor(rotations,rotationUnits::rev,false);
      RightBack.rotateFor(rotations,rotationUnits::rev,true);
  }
  return;
}
*/


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
  BotLocation = FieldLocate();
  return;
  HController.Screen.clearScreen();
  HController.Screen.setCursor(1,1);
  HController.Screen.print("Mode:");
  HController.Screen.setCursor(2,2);
  switch(BotLocation){
    default:
      HController.Screen.print("HighGoal on left");
    case 2:
      HController.Screen.print("HighGoal on Right");
    case 3:
      HController.Screen.print("No Auton");
  }/*
  HController.Screen.setCursor(3,1);
  HController.Screen.print("Status:");
  HController.Screen.setCursor(4,2);
  HController.Screen.print("Ready");*/
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
  return;
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  /*HController.Screen.clearScreen();
  HController.Screen.setCursor(1,1);
  HController.Screen.print("Mode:");
  HController.Screen.setCursor(2,2);
  switch(BotLocation){
    default:
      HController.Screen.print("HighGoal on left");
    case 2:
      HController.Screen.print("HighGoal on Right");
    case 3:
      HController.Screen.print("No Auton");
  }
  HController.Screen.setCursor(3,1);
  HController.Screen.print("Status:");
  HController.Screen.setCursor(4,2);
  if (BotLocation==3){
    HController.Screen.print("Skiping Auton...");
    return;
  }
  HController.Screen.print("Running Auton...");*/
  //Auton based on Highgoal left
  LeftFront.setTimeout(3,timeUnits::sec);
  LeftBack.setTimeout(3,timeUnits::sec);
  RightFront.setTimeout(3,timeUnits::sec);
  RightBack.setTimeout(3,timeUnits::sec);
  FWD(0.5);
  Rotate(-45);
  FWD(2);
  Rotate(90);
  Fire();
  FWD(0.075);
  Rotate(90);
  FWD(-2.5);
  indexer.rotateFor(3,rotationUnits::rev,true);
  /*HController.Screen.clearScreen();
  HController.Screen.setCursor(1,1);
  HController.Screen.print("Mode:");
  HController.Screen.setCursor(2,2);
  HController.Screen.print("Driver Control");
  HController.Screen.setCursor(3,1);
  HController.Screen.print("Status:");
  HController.Screen.setCursor(4,2);
  HController.Screen.print("Pending Start");*/
  return;

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
  /*Brain.setTimer(105,timeUnits::sec);
  // User control code here, inside the loop
  HController.Screen.clearScreen();
  HController.Screen.setCursor(1,1);
  HController.Screen.print("Mode:");
  HController.Screen.setCursor(2,2);
  HController.Screen.print("H Driver Control");
  HController.Screen.setCursor(3,1);
  HController.Screen.print("Seconds Left:");
  HController.Screen.setCursor(4,2);

  VController.Screen.clearScreen();
  VController.Screen.setCursor(1,1);
  VController.Screen.print("Mode:");
  VController.Screen.setCursor(2,2);
  VController.Screen.print("V Driver Control");
  VController.Screen.setCursor(3,1);
  VController.Screen.print("Seconds Left:");
  VController.Screen.setCursor(4,2);*/
  double val;
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    /*HController.Screen.clearLine();
    HController.Screen.print(Brain.timer(timeUnits::sec));

    VController.Screen.clearLine();
    VController.Screen.print(Brain.timer(timeUnits::sec));*/

    LeftFront.spin(directionType::fwd,HController.Axis3.value()/11.5,voltageUnits::volt);
    LeftBack.spin(directionType::fwd,HController.Axis3.value()/11.5,voltageUnits::volt);
    RightFront.spin(directionType::fwd,HController.Axis2.value()/11.5,voltageUnits::volt);
    RightBack.spin(directionType::fwd,HController.Axis2.value()/11.5,voltageUnits::volt);

    if (VController.ButtonR1.pressing()==1){
      PrimaryFly.spin(directionType::fwd,highspeed,voltageUnits::volt);
      SecondaryFly.spin(directionType::fwd,highspeed,voltageUnits::volt);
    }
    /*else if (VController.ButtonR2.pressing()==1){
      PrimaryFly.spin(directionType::fwd,lowspeed,voltageUnits::volt);
      SecondaryFly.spin(directionType::fwd,lowspeed,voltageUnits::volt);
    }*/

    if (HController.ButtonR1.pressing()==1){
      indexer.spin(directionType::fwd,100,velocityUnits::pct);
    }
    else{
    val = VController.Axis2.value()/1.27;
    indexer.spin(directionType::fwd,posfilter(val),velocityUnits::pct);
    }
    
    intake.spin(directionType::fwd,VController.Axis2.value()/1.27,velocityUnits::pct);

    if(VController.ButtonA.pressing()==1)
    {
      DigitalOutA.set(false);      
    }
    else{
      DigitalOutA.set(true);
    }

    //vex::task::sleep(20);

    wait(20, msec); // Sleep the task for a short amount of time to
    //                // prevent wasted resources.
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
    
  }
}
