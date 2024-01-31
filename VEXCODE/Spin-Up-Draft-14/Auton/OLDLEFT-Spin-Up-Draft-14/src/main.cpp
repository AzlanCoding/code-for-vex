/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       AzlanCoding                                               */
/*    Created:      Tue Apr 04 2023                                           */
/*    Description:                                                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// expand               digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <math.h>

#define _USE_MATH_DEFINES

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

motor PriFly = motor(PORT11, gearSetting::ratio18_1, true);
motor SecFly = motor(PORT16, gearSetting::ratio18_1, false);

//digital_out expand = digital_out(Brain.ThreeWirePort.A);

controller H = controller(primary);
controller V = controller(partner);

motor base[4] = {LF,RF,LB,RB};
motor Left[2] = {LF,LB};
motor Right[2] = {RF,RB};

double wheelCircumference = 4 * 2.54 * M_PI; //(( wheel(inch) * 2.54)[convert to cm] * pi)The circumference of the wheel. (cm)
double wheelTrack = 24.5 * M_PI; //(distance between left and right wheels * pi)The distance a wheel will travel to spin the bot 360deg(cm)

bool disc_intaken = true;

void VMove(double tiles, bool intake1 = false, int speed=6,voltageUnits velo=voltageUnits::volt){

  double target = ((tiles*60.96)/wheelCircumference);
  double starget = target*0.75;
  /*Brain.Screen.print(target);
  return;*/
  double actual = 0;
  directionType dir;

  if (target > 0){
    dir = directionType::fwd;
  }
  else if (target < 0){
    dir = directionType::rev;
  }
  else{
    return;
  }

  for (motor m: base){
    m.resetRotation();
  }

  if (intake1 == true && disc_intaken == false){
    intake.spin(directionType::fwd,10,voltageUnits::volt);
    indexer.spin(directionType::fwd,10,voltageUnits::volt);
    task::sleep(100);
  }

  for (motor m: base){
    m.spin(dir,speed,velo);
  }

  while (true){
    actual = 0;
    for (motor m: base){
      actual += m.rotation(rotationUnits::rev);
    }
    actual /= 4;
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print(target);
    Brain.Screen.print(" ");
    Brain.Screen.print(actual);
    if ((actual > target && target > 0)||(actual < target && target < 0)){
      break;
    }
    if ((actual > starget && starget > 0)||(actual < starget && starget < 0)){
      for (motor m: base){
        m.spin(dir,speed*0.5,velo);
      }
    }
    /*if (indexer.torque() >= 0.25 && intake1 == true && disc_intaken == false){
      //task::sleep(1000);
      intake.spin(directionType::fwd,0,voltageUnits::volt);
      indexer.spin(directionType::fwd,0,voltageUnits::volt);
      disc_intaken = true;
    }*/
  }

  for (motor m: base){
    m.spin(dir,0,voltageUnits::volt);
  }
  intake.spin(directionType::fwd,0,voltageUnits::volt);
  indexer.spin(directionType::fwd,0,voltageUnits::volt);
  return;
}


void Rotate(int deg, double speed = 5, voltageUnits velo = voltageUnits::volt){
  double pct = deg/(double)360;
  double rotations = wheelTrack / wheelCircumference * pct;
  double LTarget = rotations;
  double RTarget = rotations *-1;
  double LActual;
  double RActual;
  directionType Ldir;
  directionType Rdir;
  if (LTarget < 0 && RTarget > 0){
    Ldir = directionType::rev;
    Rdir = directionType::fwd;
  }
  else if (LTarget > 0 && RTarget < 0){
    Ldir = directionType::fwd;
    Rdir = directionType::rev;
  }
  else{
    //H.Screen.print("Error!");
    return;
  }

  for (motor m: base){
    m.resetRotation();
  }

  /*LF.spin(directionType::rev,3,voltageUnits::volt);
  task::sleep(5000);*/
  LF.spin(Ldir,speed,velo);
  RF.spin(Rdir,speed,velo);
  LB.spin(Ldir,speed,velo);
  RB.spin(Rdir,speed,velo);

  while (true){
    LActual = (LF.rotation(rotationUnits::rev) + LB.rotation(rotationUnits::rev))/2;
    RActual = (RF.rotation(rotationUnits::rev) + RB.rotation(rotationUnits::rev))/2;
    if (LTarget < 0 && RTarget > 0){
      if (LActual <= LTarget || RActual >= RTarget){
        break;
      }
    }
    else if (LTarget > 0 && RTarget < 0){
      if (LActual >= LTarget || RActual <= RTarget){
        break;
      }
    }
    task::sleep(50);
  }

  //H.Screen.print("DOn");

  for (motor m: base){
    m.spin(directionType::fwd,0,voltageUnits::volt);
  }
  task::sleep(250);
  return;

}

void LaunchSEQ(){
  int discs = 0;
  if (disc_intaken == false){
    discs++;
    //task::sleep(1000);
  }
  else{
    intake.spin(directionType::fwd,10,voltageUnits::volt);
    indexer.spin(directionType::fwd,10,voltageUnits::volt);
    task::sleep(3000);
  }
  disc_intaken = false;
  indexer.spin(directionType::rev,12,voltageUnits::volt);
  intake.spin(directionType::fwd,12,voltageUnits::volt);
  while (true){
    if (PriFly.torque() > 0.30){
      intake.spin(directionType::fwd,0,voltageUnits::volt);
      indexer.spin(directionType::fwd,0,voltageUnits::volt);
      discs ++;
      if (discs >= 3){
        task::sleep(500);
        PriFly.spin(directionType::fwd,0,voltageUnits::volt);
        SecFly.spin(directionType::fwd,0,voltageUnits::volt);
        indexer.spin(directionType::fwd,0,voltageUnits::volt);
        intake.spin(directionType::fwd,0,voltageUnits::volt);
        break;
      }
      else{
        task::sleep(3000);
        //while (PriFly.torque() > 0.30){Brain.Screen.clearScreen();Brain.Screen.print("Waiting for Flywheel Stage 1");}
        //while (PriFly.velocity(velocityUnits::pct) < 100){Brain.Screen.clearScreen();Brain.Screen.print("Waiting for Flywheel Stage 2");}
        Brain.Screen.clearScreen();
        indexer.spin(directionType::rev,12,voltageUnits::volt);
        intake.spin(directionType::fwd,12,voltageUnits::volt);
      }
    }
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
  Brain.Timer.reset();
  PriFly.spin(directionType::fwd,12,voltageUnits::volt);
  SecFly.spin(directionType::fwd,12,voltageUnits::volt);
  VMove(2,true,6);
  //if (disc_intaken == true){H.rumble(rumbleLong);Rotate(45);}
  Rotate(50);
  VMove(0.10,true);
  //VMove(0.20);
  LaunchSEQ();
  H.Screen.setCursor(1,1);
  H.Screen.clearScreen();
  H.Screen.print("Time Used: ");
  H.Screen.print(Brain.Timer.value());

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
  //autonomous();
  return;
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
  /*Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);*/

  // Run the pre-autonomous function.
  pre_auton();
  /*if (disc_intaken == false){
    intake.spin(directionType::fwd,10,voltageUnits::volt);
    indexer.spin(directionType::fwd,10,voltageUnits::volt);   
  }
  PriFly.spin(directionType::fwd,12,voltageUnits::volt);
  SecFly.spin(directionType::fwd,12,voltageUnits::volt);
  task::sleep(200);
  while (true){
    Brain.Screen.setCursor(1,1);
    Brain.Screen.clearLine();
    Brain.Screen.print(PriFly.velocity(velocityUnits::pct));
    Brain.Screen.setCursor(2,1);
    Brain.Screen.clearLine();
    Brain.Screen.print(SecFly.velocity(velocityUnits::pct));*/
    /*if (indexer.torque() >= 0.20){
      //task::sleep(1000);
      intake.spin(directionType::fwd,0,voltageUnits::volt);
      indexer.spin(directionType::fwd,0,voltageUnits::volt);  
      break;
    }*//*
    task::sleep(50);
  }*/
  //task::sleep(5000);
  //return 0;

  //VMove(0.45);
  //Brain.Screen.print("Done");

  autonomous();

  //Rotate(90);

  //H.Screen.print("Done!");

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
