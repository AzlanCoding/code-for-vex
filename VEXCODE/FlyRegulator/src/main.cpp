/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\T0825640Z                                        */
/*    Created:      Fri Apr 14 2023                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;


motor PriFly = motor(PORT11, gearSetting::ratio6_1, true);
motor SecFly = motor(PORT16, gearSetting::ratio6_1, false);
motor intake = motor(PORT5, gearSetting::ratio18_1, true);
motor indexer = motor(PORT1, gearSetting::ratio18_1, false);

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



int LaunchSEQMain(){
  //PriFly.spin(directionType::fwd,speed,velocityUnits::rpm);
  //SecFly.spin(directionType::fwd,speed,velocityUnits::rpm);
  PriFly.spin(directionType::fwd,12,voltageUnits::volt);
  SecFly.spin(directionType::fwd,12,voltageUnits::volt);
  Brain.Screen.clearScreen();
  WaitFly();
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
      intake.spin(directionType::fwd,0,voltageUnits::volt);
      indexer.spin(directionType::fwd,0,voltageUnits::volt);
      disc++;
      /*float a = PriFly.velocity(velocityUnits::rpm);
      Brain.Screen.setCursor(1,1);
      Brain.Screen.clearLine();
      Brain.Screen.print(a);
      while (true){}*/

      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1,1);
      Brain.Screen.clearLine();
      Brain.Screen.print("Discs: ");
      Brain.Screen.print(disc);
      task::sleep(1250);
      if (disc >= 3){
        intake.spin(directionType::fwd,0,voltageUnits::volt);
        indexer.spin(directionType::fwd,0,voltageUnits::volt);
        PriFly.spin(directionType::fwd,0,voltageUnits::volt);
        SecFly.spin(directionType::fwd,0,voltageUnits::volt);
        break;
      }
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
      indexer.stop(brakeType::hold);
      intake.stop(brakeType::hold);
      //intake.spin(directionType::rev,0,voltageUnits::volt);
      //indexer.spin(directionType::fwd,0,voltageUnits::volt);
      //task::sleep(3000);
      //while(PriFly.velocity(velocityUnits::rpm) < speed){Brain.Screen.setCursor(1,1);Brain.Screen.clearLine();Brain.Screen.print(PriFly.velocity(velocityUnits::rpm));}
      task::sleep(1250);
      if (disc >= 3){
        intake.spin(directionType::fwd,0,voltageUnits::volt);
        indexer.spin(directionType::fwd,0,voltageUnits::volt);
        PriFly.spin(directionType::fwd,0,voltageUnits::volt);
        SecFly.spin(directionType::fwd,0,voltageUnits::volt);
        break;
      }
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


int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  LaunchSEQ();

  vexSystemExitRequest();
}
