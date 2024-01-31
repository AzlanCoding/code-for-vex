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
// Expansion            digital_out   C               
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

motor cata = motor(PORT11, gearSetting::ratio36_1, false);

//digital_in limitSwitch = digital_in(Brain.ThreeWirePort.A);

motor base[6] = {LF,LM,LB,RF,RM,RB};

bool overide = false;
bool slwDown;
double wheelCircumference = 4 * 2.54 * M_PI; //(( wheel(inch) * 2.54)[convert to cm] * pi)The circumference of the wheel. (cm)
double wheelTrack = 38.2 * M_PI; //(distance between left and right wheels * pi)The distance a wheel will travel to spin the bot 360deg(cm)


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
  Expansion.set(false);

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

bool slowDown(double initial, double target){
  return false;
  double diff = target - initial;
  if (target < 0){
    diff = diff * -1;
  }
  /*if (slwDown == false){
    Brain.Screen.setCursor(4,1);
    Brain.Screen.clearLine();
    Brain.Screen.print(target);
    Brain.Screen.print(" ");
    Brain.Screen.print(initial);
    Brain.Screen.print(" ");
    Brain.Screen.print(diff);
  }*/
  if (diff <= 0.15){
    /*if (slwDown == false){
      Brain.Screen.print(" ");
      Brain.Screen.print("trig at ");
      Brain.Screen.print(diff);
    }*/
    return true;
  }
  return false;
}

directionType Check(double val){
  if (val >= 0){
    return directionType::fwd;
  }
  else{
    return directionType::rev;
  }
}

void Move(double left, double right){
  slwDown = false;
  double plan[6] = {left,left,left,right,right,right};
  bool turnin;
  //bool finished = false;
  bool status[6] = {false,false,false,false,false,false};
  bool slwDownList[6] = {false,false,false,false,false,false};
  int statusCount;
  if (left == right){
    turnin = false;
  } else{
    turnin = true;
  }

  for (int i=0; i<=6; i++){
    base[i].resetRotation();
    if (turnin == true){
      base[i].spin(Check(plan[i]),5,voltageUnits::volt);
    }
    else{
      base[i].spin(Check(plan[i]),5,voltageUnits::volt);
    }
  }
  while (true){
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    for (int i=0; i<=6; i++){
      Brain.Screen.print(base[i].rotation(rotationUnits::rev));
      Brain.Screen.newLine();
      if (base[i].rotation(rotationUnits::rev) >= plan[i]){
        base[i].spin(directionType::fwd,0,voltageUnits::volt);
        status[i] = true;
      } else if (slowDown(base[i].rotation(rotationUnits::rev),plan[i]) && slwDownList[i] == false){
        slwDownList[i] = true;
        base[i].spin(directionType::fwd,5,voltageUnits::volt);
      }
    }
    statusCount = 0;
    for (bool check :status){
      if (check == true){
        statusCount++;
      }
    }
    if (statusCount == 6){
      //finished = true;
      break;
    }
  }

}

void Stop(){
  for (motor m :base){
    m.spin(directionType::fwd,0,voltageUnits::volt);
  }
}

void FWD(int r){
  Move(r,r);
}

void rotate(double deg){
  double pct;
  pct = deg/(double)360;
  double rotations;
  rotations = wheelTrack / wheelCircumference * pct;
  Move(rotations,rotations*-1);
}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  /*FWD(1);
  //rotate(90);
  Stop();
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  for (int i=0; i<=6; i++){
    Brain.Screen.print(base[i].rotation(rotationUnits::rev));
    Brain.Screen.newLine();
  }
  Brain.Screen.print("AutonDone");*/
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
  //H.Screen.clearScreen();
  V.Screen.clearScreen();
  //H.Screen.setCursor(1,1);
  V.Screen.setCursor(1,1);
  //H.Screen.print("Override: False");
  V.Screen.print("Override: False");
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    LF.spin(directionType::fwd, H.Axis3.value()/10.583, voltageUnits::volt);
    LM.spin(directionType::fwd, H.Axis3.value()/10.583, voltageUnits::volt);
    LB.spin(directionType::fwd, H.Axis3.value()/10.583, voltageUnits::volt);
    RF.spin(directionType::fwd, H.Axis2.value()/10.583, voltageUnits::volt);
    RM.spin(directionType::fwd, H.Axis2.value()/10.583, voltageUnits::volt);
    RB.spin(directionType::fwd, H.Axis2.value()/10.583, voltageUnits::volt);

    if (limitSwitch.value()==1 || overide == true){
      intake.spin(directionType::fwd, V.Axis3.value()/10.583, voltageUnits::volt);
    }
    else if (V.Axis3.value()!= 0) {
      //V.Screen.clearScreen();
      //V.Screen.print("Cata not down");
      //V.Screen.print("Press 'R1' to overide");
      V.rumble("-");
    }

    if (H.ButtonL2.pressing()==1){
      Expansion.set(true);
      while (H.ButtonL2.pressing()==1){
        //wait for button to release
      }
      Expansion.set(false);
    }
    if (H.ButtonL1.pressing()==1){
      Expansion.set(true);
      while (H.ButtonL1.pressing()==1){
        //wait for button to release
      }
      Expansion.set(false);
    }

    if (V.ButtonA.pressing()==1){
      //V.rumble(".");
      if (overide == false){
        //H.Screen.clearScreen();
        V.Screen.clearScreen();
        //H.Screen.setCursor(1,1);
        V.Screen.setCursor(1,1);
        //H.Screen.print("Override: True");
        V.Screen.print("Override: True");
        overide = true;
      }
      else{
        //H.Screen.clearScreen();
        V.Screen.clearScreen();
        //H.Screen.setCursor(1,1);
        V.Screen.setCursor(1,1);
        //H.Screen.print("Override: False");
        V.Screen.print("Override: False");
        overide = false;
      }
      while (V.ButtonA.pressing()==1){
        //wait for button to release
      }
    }

    if (V.ButtonR2.pressing()==1 /*&& cata.isSpinning()==0 */&& limitSwitch.pressing()==0){//Arm down
      cata.spin(directionType::fwd, 10.583, voltageUnits::volt);
      while (limitSwitch.pressing()==0){
        if (V.ButtonA.pressing()==1){
          //V.rumble(".");
          if (overide == false){
            //H.Screen.clearScreen();
            V.Screen.clearScreen();
            //H.Screen.setCursor(1,1);
            V.Screen.setCursor(1,1);
            //H.Screen.print("Override: True");
            V.Screen.print("Override: True");
            overide = true;
          }
          else{
            //H.Screen.clearScreen();
            V.Screen.clearScreen();
            //H.Screen.setCursor(1,1);
            V.Screen.setCursor(1,1);
            //H.Screen.print("Override: False");
            V.Screen.print("Override: False");
            overide = false;
          }
          while (V.ButtonA.pressing()==1){
            //wait for button to release
          }
          break;
        }
        LF.spin(directionType::fwd, H.Axis3.value()/10.583, voltageUnits::volt);
        LM.spin(directionType::fwd, H.Axis3.value()/10.583, voltageUnits::volt);
        LB.spin(directionType::fwd, H.Axis3.value()/10.583, voltageUnits::volt);
        RF.spin(directionType::fwd, H.Axis2.value()/10.583, voltageUnits::volt);
        RM.spin(directionType::fwd, H.Axis2.value()/10.583, voltageUnits::volt);
        RB.spin(directionType::fwd, H.Axis2.value()/10.583, voltageUnits::volt);
        if (H.ButtonL2.pressing()==1){
          Expansion.set(true);
          while (H.ButtonL2.pressing()==1){
            //wait for button to release
          }
          Expansion.set(false);
        }
        if (H.ButtonL1.pressing()==1){
          Expansion.set(true);
          while (H.ButtonL1.pressing()==1){
            //wait for button to release
          }
          Expansion.set(false);
        }
      }
      cata.spin(directionType::fwd, 0, voltageUnits::volt);
    } 
    else if ((V.ButtonR1.pressing()==1 || H.ButtonR1.pressing()==1 ) && limitSwitch.pressing()==1 /*&& cata.isSpinning()==0*/){
      cata.spin(directionType::fwd, 10.583, voltageUnits::volt);
      while (limitSwitch.pressing()==1){
        if (V.ButtonA.pressing()==1){
          //V.rumble(".");
          if (overide == false){
            //H.Screen.clearScreen();
            V.Screen.clearScreen();
            //H.Screen.setCursor(1,1);
            V.Screen.setCursor(1,1);
            //H.Screen.print("Override: True");
            V.Screen.print("Override: True");
            overide = true;
          }
          else{
            //H.Screen.clearScreen();
            V.Screen.clearScreen();
            //H.Screen.setCursor(1,1);
            V.Screen.setCursor(1,1);
            //H.Screen.print("Override: False");
            V.Screen.print("Override: False");
            overide = false;
          }
          while (V.ButtonA.pressing()==1){
            //wait for button to release
          }
          break;
        }
        LF.spin(directionType::fwd, H.Axis3.value()/10.583, voltageUnits::volt);
        LM.spin(directionType::fwd, H.Axis3.value()/10.583, voltageUnits::volt);
        LB.spin(directionType::fwd, H.Axis3.value()/10.583, voltageUnits::volt);
        RF.spin(directionType::fwd, H.Axis2.value()/10.583, voltageUnits::volt);
        RM.spin(directionType::fwd, H.Axis2.value()/10.583, voltageUnits::volt);
        RB.spin(directionType::fwd, H.Axis2.value()/10.583, voltageUnits::volt);
        intake.spin(directionType::fwd, 0, voltageUnits::volt);
        if (H.ButtonL2.pressing()==1){
          Expansion.set(true);
          while (H.ButtonL2.pressing()==1){
            //wait for button to release
          }
          Expansion.set(false);
        }
        if (H.ButtonL1.pressing()==1){
          Expansion.set(true);
          while (H.ButtonL1.pressing()==1){
            //wait for button to release
          }
          Expansion.set(false);
        }
      }
      cata.spin(directionType::fwd, 0, voltageUnits::volt);
    } 
    else {
      //V.Screen.clearScreen();
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
  //autonomous();
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  //autonomous();

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
