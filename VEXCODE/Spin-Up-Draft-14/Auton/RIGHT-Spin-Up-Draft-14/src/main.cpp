/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       AzlanCoding                                               */
/*    Created:      Wed Mar 29 2023                                           */
/*    Description:                                                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// expand               digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----
bool testing = false;

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

motor PriFly = motor(PORT11, gearSetting::ratio6_1, true);
motor SecFly = motor(PORT16, gearSetting::ratio6_1, false);

//digital_out expand = digital_out(Brain.ThreeWirePort.A);

controller H = controller(primary);
controller V = controller(partner);

motor base[4] = {LF,RF,LB,RB};
motor Left[2] = {LF,LB};
motor Right[2] = {RF,RB};

double wheelCircumference = 4 * 2.54 * M_PI; //(( wheel(inch) * 2.54)[convert to cm] * pi)The circumference of the wheel. (cm)
double wheelTrack = 24.5 * M_PI; //(distance between left and right wheels * pi)The distance a wheel will travel to spin the bot 360deg(cm)
double wheelCircumference2 = 4 * M_PI; //(( wheel(inch) * 2.54)[convert to cm] * pi)The circumference of the wheel. (cm)
double wheelTrack2 = 12.1; //(distance between left and right wheels * pi)The distance a wheel will travel to spin the bot 360deg(cm)

bool disc_intaken = true;

float offset = 0;

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
    if (indexer.torque() >= 0.25 && intake1 == true && disc_intaken == false){
      //task::sleep(1000);
      intake.spin(directionType::fwd,0,voltageUnits::volt);
      indexer.spin(directionType::fwd,0,voltageUnits::volt);
      disc_intaken = true;
    }
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

void LaunchSEQOld(){
  int discs = 0;
  if (disc_intaken == false){
    discs++;
    task::sleep(1000);
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
    if (PriFly.torque() > 0.12){
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














int corrections = 0;
bool prevDir = false;//tru if reversing
void prevDi(bool a){
    if (prevDir == a){
      return;
    }
    else{
      prevDir = a;
      corrections++;
      return;
    }
}

void Base(float speed){
  RF.spin(directionType::fwd,speed,voltageUnits::volt);
  LF.spin(directionType::fwd,speed,voltageUnits::volt);
  LB.spin(directionType::fwd,speed,voltageUnits::volt);
  RB.spin(directionType::fwd,speed,voltageUnits::volt);
}

void baseR(float speed){
  LF.spin(directionType::fwd,speed*-1,voltageUnits::volt);
  LB.spin(directionType::fwd,speed*-1,voltageUnits::volt);
  RF.spin(directionType::fwd,speed,voltageUnits::volt);
  RB.spin(directionType::fwd,speed,voltageUnits::volt);
}

void baseL(float speed){
  LF.spin(directionType::fwd,speed,voltageUnits::volt);
  LB.spin(directionType::fwd,speed,voltageUnits::volt);
  RF.spin(directionType::fwd,speed*-1,voltageUnits::volt);
  RB.spin(directionType::fwd,speed*-1,voltageUnits::volt);
}

void VMove2(double target, int speed){
  double revs = 0;
  LF.resetRotation();
  LB.resetRotation();
  RF.resetRotation();
  RB.resetRotation();
  while (revs <= target){
    revs = (LF.rotation(rotationUnits::rev)+LB.rotation(rotationUnits::rev)+RF.rotation(rotationUnits::rev)+RB.rotation(rotationUnits::rev))/4;
    LF.spin(directionType::fwd,speed,voltageUnits::volt);
    LB.spin(directionType::fwd,speed,voltageUnits::volt);
    RF.spin(directionType::fwd,speed,voltageUnits::volt);
    RB.spin(directionType::fwd,speed,voltageUnits::volt);
  }
  LF.spin(directionType::fwd,0,voltageUnits::volt);
  LB.spin(directionType::fwd,0,voltageUnits::volt);
  RF.spin(directionType::fwd,0,voltageUnits::volt);
  RB.spin(directionType::fwd,0,voltageUnits::volt);
}



void Rotate2(float deg){
  corrections = 0;
  prevDir = false;//tru if reversing
  double Wheel = wheelCircumference2;//Wheel Circumference in inches
  double TRad = deg*(M_PI/180);//target in radians
  double dist = wheelTrack2;//distance between left and right wheel in inches
  double CRad = 0;//current heading in radians
  float L = 0;
  float R = 0;
  float pctTar = 0;
  float MinSped = 3.0;
  float MaxSped = (11.5 - MinSped);
  float sped = 0;
  LF.resetRotation();
  LB.resetRotation();
  RF.resetRotation();
  RB.resetRotation();
  while(CRad != TRad&&corrections <= 6){
    L = (((LF.rotation(rev)*Wheel)+(LB.rotation(rev)*Wheel))/2);
    R = (((RF.rotation(rev)*Wheel)+(RB.rotation(rev)*Wheel))/2);
    CRad = (R-L)/dist;
    pctTar = 1 - (CRad/TRad);
    sped = (MaxSped*pctTar)+ MinSped;
    if (deg > 0){
      if (CRad < TRad){
        baseR(sped);
        prevDi(false);
      }
      else if(CRad > TRad){
        baseL(MinSped);
        prevDi(true);
      }
    }
    else if (deg < 0){
      if (CRad > TRad){
        baseL(sped);
        prevDi(false);
      }
      else if(CRad < TRad){
        baseR(MinSped);
        prevDi(true);
      }
    }
    else{
      break;
    }
    task::sleep(5);
  }
  Base(0);
}

void AvdFwd(double tar,bool align = false){
  corrections = 0;
  prevDir = false;//tru if reversing
  offset = 0;
  LF.resetRotation();
  LB.resetRotation();
  RF.resetRotation();
  RB.resetRotation();
  double err = tar;
  float pctTar = 0;
  float MinSped = 6;
  float MaxSped = (12 - MinSped);
  float sped = 0;
  while (err != 0&&corrections <= 5){
    err = tar - ((LF.rotation(rev)+LB.rotation(rev)+RF.rotation(rev)+RB.rotation(rev))/4);
    pctTar = 1-((LF.rotation(rev)+LB.rotation(rev)+RF.rotation(rev)+RB.rotation(rev))/4)/tar;
    sped = (MaxSped*pctTar)+ MinSped;
    if (err > 0){
      Base(sped);
      prevDi(false);
    }
    else if (err < 0){
      Base((MinSped*-1)-3);
      prevDi(true);
    }
  }
  Base(0);
  if (align==1){
    float L = (((LF.rotation(rev)*wheelCircumference2)+(LB.rotation(rev)*wheelCircumference2))/2);
    float R = (((RF.rotation(rev)*wheelCircumference2)+(RB.rotation(rev)*wheelCircumference2))/2);
    if ((R-L)/wheelTrack2 != 0){
      H.rumble(rumbleShort);
      double deg = ((R-L)/wheelTrack2)*(180/M_PI);
      H.Screen.print(deg);
      offset = 0;
    }
  }
  return;
}


void AvdRev(double tar, bool align = false){
  corrections = 0;
  prevDir = false;//tru if reversing
  offset = 0;
  LF.resetRotation();
  LB.resetRotation();
  RF.resetRotation();
  RB.resetRotation();
  double err = tar;
  float pctTar = 0;
  float MinSped = 3;
  float MaxSped = (11.5 - MinSped);
  float sped = 0;
  while (err != tar&&corrections <= 10){
    err = tar - ((LF.rotation(rev)+LB.rotation(rev)+RF.rotation(rev)+RB.rotation(rev))/4);
    pctTar = ((LF.rotation(rev)+LB.rotation(rev)+RF.rotation(rev)+RB.rotation(rev))/4)/tar;
    sped = (MaxSped*pctTar)+ MinSped;
    if (err < 0){
      Base(sped*-1);
      prevDi(false);
    }
    else if (err > 0){
      Base(MinSped);
      prevDi(true);
    }
  }
  Base(0);
  if (align==1){
    float L = (((LF.rotation(rev)*wheelCircumference2)+(LB.rotation(rev)*wheelCircumference2))/2);
    float R = (((RF.rotation(rev)*wheelCircumference2)+(RB.rotation(rev)*wheelCircumference2))/2);
    if ((R-L)/wheelTrack2 != 0){
      H.rumble(rumbleShort);
      double deg = ((R-L)/wheelTrack2)*(180/M_PI);
      //Rotate2(deg);
      offset = deg;
    }
  }
}







float speed=600;
float Launchspeed = 575;
int disc = 0;
void WaitFly(){
  int i = 0;
  //bool launched = false;
  intake.spin(directionType::fwd,0,voltageUnits::volt);
  indexer.spin(directionType::fwd,0,voltageUnits::volt);
  while(true){
    /*if (PriFly.torque() > 0.105 && launched == false){
      disc++;
      launched = true;
    }*/
    if (PriFly.velocity(velocityUnits::rpm) > speed){
      i++;
    }
    else if (PriFly.velocity(velocityUnits::rpm) < speed){
      i = 0;
    }
    if (i >= 4){
      break;
    }
    else{
      task::sleep(50);
    }
    Brain.Screen.setCursor(1,1);
    Brain.Screen.clearLine();
    Brain.Screen.print(PriFly.velocity(velocityUnits::rpm));
  }
}


int CounDisk(){
  while (disc <= 3){
    if (PriFly.torque() > 0.12){
      disc++;
      task::sleep(1250);
    }
  }
  return 0;
}


int LaunchSEQMain(){
  //PriFly.spin(directionType::fwd,speed,velocityUnits::rpm);
  //SecFly.spin(directionType::fwd,speed,velocityUnits::rpm);
  PriFly.spin(directionType::fwd,12,voltageUnits::volt);
  SecFly.spin(directionType::fwd,12,voltageUnits::volt);
  Brain.Screen.clearScreen();
  WaitFly();
  task DiscCounter = task(CounDisk);
  intake.spin(directionType::fwd,12,voltageUnits::volt);
  indexer.spin(directionType::rev,12,voltageUnits::volt);
  while (true){
    Brain.Screen.setCursor(1,1);
    Brain.Screen.clearLine();
    Brain.Screen.print(PriFly.velocity(velocityUnits::rpm));
    if (Brain.Screen.pressing()){
      intake.spin(directionType::fwd,0,voltageUnits::volt);
      indexer.spin(directionType::fwd,0,voltageUnits::volt);
      PriFly.spin(directionType::fwd,0,voltageUnits::volt);
      SecFly.spin(directionType::fwd,0,voltageUnits::volt);
      while (Brain.Screen.pressing()){}
      PriFly.spin(directionType::fwd,12,voltageUnits::volt);
      SecFly.spin(directionType::fwd,12,voltageUnits::volt);
    }
    if (PriFly.torque() > 0.12){
      task::sleep(50);
      intake.spin(directionType::fwd,0,voltageUnits::volt);
      indexer.spin(directionType::fwd,0,voltageUnits::volt);
      //disc++;
      /*float a = PriFly.velocity(velocityUnits::rpm);
      Brain.Screen.setCursor(1,1);
      Brain.Screen.clearLine();
      Brain.Screen.print(a);
      while (true){}*/

      /*Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1,1);
      Brain.Screen.clearLine();
      Brain.Screen.print("Discs: ");
      Brain.Screen.print(disc);*/
      if (disc >= 3){
        intake.spin(directionType::fwd,0,voltageUnits::volt);
        indexer.spin(directionType::fwd,0,voltageUnits::volt);
        PriFly.spin(directionType::fwd,0,voltageUnits::volt);
        SecFly.spin(directionType::fwd,0,voltageUnits::volt);
        break;
      }
      task::sleep(700);
      WaitFly();
      //task::sleep(3000);
      intake.spin(directionType::fwd,12,voltageUnits::volt);
      indexer.spin(directionType::rev,12,voltageUnits::volt);
      //indexer.rotateFor(directionType::rev,1.5,rotationUnits::rev,100,velocityUnits::pct);
      //task::sleep(500);
    }
    else if(PriFly.velocity(velocityUnits::rpm) < speed && PriFly.velocity(velocityUnits::rpm) > Launchspeed){
      /*if (PriFly.torque() > 0.12){
        disc++;
      }*/
      //disc++;
      /*float a = PriFly.velocity(velocityUnits::rpm);
      Brain.Screen.setCursor(1,1);
      Brain.Screen.clearLine();
      Brain.Screen.print(a);
      while (true){}*/
      task::sleep(50);
      indexer.stop(brakeType::hold);
      intake.stop(brakeType::hold);
      //intake.spin(directionType::rev,0,voltageUnits::volt);
      //indexer.spin(directionType::fwd,0,voltageUnits::volt);
      //task::sleep(3000);
      //while(PriFly.velocity(velocityUnits::rpm) < speed){Brain.Screen.setCursor(1,1);Brain.Screen.clearLine();Brain.Screen.print(PriFly.velocity(velocityUnits::rpm));}
      if (disc >= 3){
        intake.spin(directionType::fwd,0,voltageUnits::volt);
        indexer.spin(directionType::fwd,0,voltageUnits::volt);
        PriFly.spin(directionType::fwd,0,voltageUnits::volt);
        SecFly.spin(directionType::fwd,0,voltageUnits::volt);
        break;
      }
      task::sleep(700);
      WaitFly();
      indexer.stop(brakeType::coast);
      intake.stop(brakeType::coast);
      intake.spin(directionType::fwd,12,voltageUnits::volt);
      indexer.spin(directionType::rev,12,voltageUnits::volt);
      //indexer.rotateFor(directionType::rev,1.5,rotationUnits::rev,100,velocityUnits::pct);
      //task::sleep(500);
    }
    if (disc >= 3){
      intake.spin(directionType::fwd,0,voltageUnits::volt);
      indexer.spin(directionType::fwd,0,voltageUnits::volt);
      PriFly.spin(directionType::fwd,0,voltageUnits::volt);
      SecFly.spin(directionType::fwd,0,voltageUnits::volt);
      break;
    }
  }
  return 0;
}

void LaunchSEQ(){
  task myTask = task(LaunchSEQMain);
  task::sleep(8000);
  disc = 4;
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
  disc =0;
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  //return;
  for (motor m: base){
    m.spin(directionType::rev,6,voltageUnits::volt);
  }
  task::sleep(200);
  for (motor m: base){
    m.stop(brakeType::hold);
  }
  //return;
  indexer.resetRotation();
  indexer.spin(directionType::fwd,12,voltageUnits::volt);
  while (indexer.rotation(rotationUnits::rev) < 1.50){Brain.Screen.clearScreen();Brain.Screen.print(indexer.rotation(rotationUnits::rev));}
  indexer.spin(directionType::fwd,0,voltageUnits::volt);
  task::sleep(250);
  intake.spin(directionType::fwd,12,voltageUnits::volt);
  indexer.spin(directionType::fwd,12,voltageUnits::volt);
  //AvdFwd(0.10);
  Rotate2(45);
  AvdFwd(0.25);
  //task::sleep(200);
  Rotate2(-45);
  AvdRev(-0.5,true);
  Rotate2(-45);
  intake.spin(directionType::fwd,0,voltageUnits::volt);
  indexer.spin(directionType::fwd,0,voltageUnits::volt);
  //task::sleep(250);
  AvdFwd(5.35,true);
  Rotate2(90+offset);
  PriFly.spin(directionType::fwd,12,voltageUnits::volt);
  SecFly.spin(directionType::fwd,12,voltageUnits::volt);
  intake.spin(directionType::fwd,12,voltageUnits::volt);
  indexer.spin(directionType::fwd,12,voltageUnits::volt);
  AvdFwd(0.40);
  LaunchSEQ();
  //Brain.Screen.print("hi");
  return;
  /*
  Brain.Timer.reset();
  for (motor m: base){
    m.spin(directionType::rev,4,voltageUnits::volt);
  }
  task::sleep(400);
  for (motor m: base){
    m.stop(brakeType::hold);
  }
  indexer.resetRotation();
  indexer.spin(directionType::fwd,12,voltageUnits::volt);
  while (indexer.rotation(rotationUnits::rev) < 1.65){Brain.Screen.clearScreen();Brain.Screen.print(indexer.rotation(rotationUnits::rev));}
  indexer.spin(directionType::fwd,0,voltageUnits::volt);
  for (motor m: base){
    m.stop(brakeType::coast);
  }
  VMove(0.20);
  //task::sleep(250);
  Rotate(65);
  PriFly.spin(directionType::fwd,11.85,voltageUnits::volt);
  SecFly.spin(directionType::fwd,11.85,voltageUnits::volt);
  VMove(2.75,true,11);
  if (disc_intaken == true){H.rumble(rumbleLong);Rotate(-120);}
  else{Rotate(-120);}
  //VMove(0.20);
  LaunchSEQ();
  H.Screen.setCursor(1,1);
  H.Screen.clearScreen();
  H.Screen.print("Time Used: ");
  H.Screen.print(Brain.Timer.value());*/

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
  if (testing){
    Brain.Timer.reset();
    autonomous();
    H.Screen.setCursor(1,1);
    H.Screen.clearScreen();
    H.Screen.print("Time Used: ");
    H.Screen.print(Brain.Timer.value());
    task::sleep(10000);
    vexSystemExitRequest();
    //return;
  }
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
  //autonomous();
  //vexSystemExitRequest();
  /*task::sleep(3000);
  //AvdFwd(4);
  Rotate2(-90);
  //LaunchSEQ();
  
  Brain.Screen.print("Done");*/




  /*
  PriFly.spin(directionType::fwd,12,voltageUnits::volt);
  SecFly.spin(directionType::fwd,12,voltageUnits::volt);
  task::sleep(200);
  while (true){
    Brain.Screen.setCursor(1,1);
    Brain.Screen.clearLine();
    Brain.Screen.print(PriFly.velocity(velocityUnits::pct));
    Brain.Screen.setCursor(2,1);
    Brain.Screen.clearLine();
    Brain.Screen.print(SecFly.velocity(velocityUnits::pct));
  }
  */

  
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  
  pre_auton();/*
  autonomous();
  vexSystemExitRequest();*/
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

  //autonomous();

  //Rotate(90);

  //H.Screen.print("Done!");

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
