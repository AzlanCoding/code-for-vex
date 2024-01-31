/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       AzlanCoding                                               */
/*    Created:      Wed Mar 29 2023                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LF                   motor         1               
// LB                   motor         2               
// RF                   motor         3               
// RB                   motor         4               
// Controller1          controller                    
// Motor                motor         5               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
#include <iostream>

using namespace vex;
using namespace std;



motor Base[4] = {RF,LF,LB,RB};
motor R[2] = {RF,RB};
motor L[2] = {LF,LB};

//void move()

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  while (true){

  }
  
}

void base(float speed){
  LF.spin(directionType::fwd,speed,voltageUnits::volt);
  LB.spin(directionType::fwd,speed,voltageUnits::volt);
  RF.spin(directionType::fwd,speed,voltageUnits::volt);
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

void VMove(double target, int speed){
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




void AvdFwd(double tar){
  LF.resetRotation();
  LB.resetRotation();
  RF.resetRotation();
  RB.resetRotation();
  double err = tar;
  float pctTar = 0;
  float MinSped = 2.0;
  float MaxSped = (11.0 - MinSped);
  float sped = 0;
  while (err != tar){
    err = tar - ((LF.rotation(rev)+LB.rotation(rev)+RF.rotation(rev)+RB.rotation(rev))/4);
    pctTar = ((LF.rotation(rev)+LB.rotation(rev)+RF.rotation(rev)+RB.rotation(rev))/4)/tar;
    sped = (MaxSped*pctTar)+ MinSped;
    if (err > 0){
      base(sped);
    }
    else if (err < 0){
      base(MinSped*-1);
    }
  }
  base(0);
}

void AvdRev(double tar){
  LF.resetRotation();
  LB.resetRotation();
  RF.resetRotation();
  RB.resetRotation();
  double err = tar;
  float pctTar = 0;
  float MinSped = 2.0;
  float MaxSped = (11.0 - MinSped);
  float sped = 0;
  while (err != tar){
    err = tar - ((LF.rotation(rev)+LB.rotation(rev)+RF.rotation(rev)+RB.rotation(rev))/4);
    pctTar = ((LF.rotation(rev)+LB.rotation(rev)+RF.rotation(rev)+RB.rotation(rev))/4)/tar;
    sped = (MaxSped*pctTar)+ MinSped;
    if (err < 0){
      base(sped*-1);
    }
    else if (err > 0){
      base(MinSped);
    }
  }
  base(0);
}

void Rotate(float deg){
  double Wheel = 2.75;//Wheel Circumference in inches
  double TRad = deg*(M_PI/180);//target in radians
  double dist = 23;//distance between left and right wheel in inches
  double CRad = 0;//current heading in radians
  double L = 0;
  double R = 0;
  double CL = 0;
  double CR = 0;
  float pctTar = 0;
  float MinSped = 2.0;
  float MaxSped = (11.0 - MinSped);
  float sped = 0;
  while(CRad != TRad){
    CL = (((LF.rotation(rev)*Wheel)+(LB.rotation(rev)*Wheel))/2)-L;
    CR = (((RF.rotation(rev)*Wheel)+(RB.rotation(rev)*Wheel))/2)-R;
    L=CL;
    R=CR;
    CRad += (CR-CL)/dist;
    pctTar = CRad/TRad;
    sped = (MaxSped*pctTar)+ MinSped;
    if (deg > 0){
      if (CRad < TRad){
        baseR(sped);
      }
      else if(CRad > TRad){
        baseL(MinSped);
      }
    }
    else if (deg < 0){
      if (CRad < TRad){
        baseL(sped);
      }
      else if(CRad > TRad){
        baseR(MinSped);
      }
    }
    else{
      break;
    }
    task::sleep(5);
  }
  base(0);
}

/*void VMove(double target, float speed=10){
  for (motor m: Base){ 
    m.resetRotation();
  }
  double val = 0;
  double Pct = exp(-(abs(val-target)/target));//Mean Percentage Error

  while (true){
    if (true){}
  }
  


}*/