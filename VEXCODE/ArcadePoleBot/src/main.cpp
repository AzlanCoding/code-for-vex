/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\T0825640Z                                        */
/*    Created:      Mon May 29 2023                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>

using namespace vex;

motor L = motor(PORT1,gearSetting::ratio18_1,false);
motor R = motor(PORT10,gearSetting::ratio18_1,true);

controller H = controller(primary);

int head =0;
double wheelCircumference2 = 4 * M_PI; //(( wheel(inch) * 2.54)[convert to cm] * pi)The circumference of the wheel. (cm)
double wheelTrack2 = 11.8; //distance between left and right wheels(inch)


/*int headin(){
  double l;
  double r;
  while(true){
    l = L.rotation(rotationUnits::rev)*wheelCircumference2;
    r = R.rotation(rotationUnits::rev)*wheelCircumference2;
    head = ((r-l)/wheelTrack2) * (180/M_PI);
  }
  return 0;
}*/


int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  int Lturn;
  int Rturn;
  int turnin;
  int threshold = 9;
  double maxSpeed = 127/12;
  int xNew;
  int yNew;
  L.resetRotation();
  R.resetRotation();
  int x;
  int y;
  int q;
  float Lt;
  float Rt;
  //task headTrack = task(headin);
  double l;
  double r;
  double turnSped;

  while (true){
    l = L.rotation(rotationUnits::rev)*wheelCircumference2;
    r = R.rotation(rotationUnits::rev)*wheelCircumference2;
    head = ((l-r)/wheelTrack2) * (180/M_PI);
    while (head > 360){
      head -= 360;
    }
    while (head < -360){
      head += 360;
    }
    if (abs(head)== 360){
      head = 0;
    }
    if (head < 0 && head > -360){
      head = 360 + head;
    }
    /*H.Screen.clearScreen();
    H.Screen.setCursor(1,1);
    H.Screen.print(head);*/
    if (H.ButtonA.pressing()){
      L.resetRotation();
      R.resetRotation();
      while(H.ButtonA.pressing()){}
    }

    if (H.Axis1.value()!=0 || H.Axis2.value()!=0){
      xNew = H.Axis1.value() * cos(head*(M_PI/180)) - H.Axis2.value() * sin(head*(M_PI/180));
      yNew = H.Axis2.value() * cos(head*(M_PI/180)) + H.Axis1.value() * sin(head*(M_PI/180));
      if (xNew==0){
        Lturn = yNew/maxSpeed;
        Rturn = yNew/maxSpeed;
      }
      else if (xNew>0 && yNew!=0){
        turnin = abs(xNew)/(127/threshold);
        Lturn = yNew/maxSpeed;
        if (yNew>0){
          Rturn = (yNew/maxSpeed)-turnin;
        }
        else{
          Rturn = (yNew/maxSpeed)+turnin;
        }
      }
      else if (xNew<0 && yNew!=0){
        turnin = abs(xNew)/(127/threshold);
        Rturn = yNew/maxSpeed;
        if (yNew>0){
          Lturn = (yNew/maxSpeed)-turnin;
        }
        else{
          Lturn = (yNew/maxSpeed)+turnin;
        }
      }
      else{
        Lturn = 0;
        Rturn = 0;
      }
      L.spin(directionType::fwd,Lturn,voltageUnits::volt);
      R.spin(directionType::fwd,Rturn,voltageUnits::volt);
    }
    else if (H.Axis3.value()!=0 || H.Axis4.value()!=0){
      //L.spin(directionType::fwd,0,voltageUnits::volt);
      //R.spin(directionType::fwd,0,voltageUnits::volt);
      x = H.Axis4.value();
      y = H.Axis3.value();
      q = atan2(x,y)* (180/M_PI);
      if (q < 0){
        q = 180 + (180-abs(q));
      }
      /*H.Screen.clearScreen();
      H.Screen.setCursor(1,1);
      //H.Screen.clearLine();
      H.Screen.print(q);
      H.Screen.setCursor(1,5);
      //H.Screen.clearLine();
      H.Screen.print(head);*/
      Rt = q - head;
      if (Rt < 0){
        Rt = 360 + Rt;
      }
      Lt = head- q;
      if (Lt < 0){
        Lt = 360 + Lt;
      }
      /*H.Screen.setCursor(2,1);
      //H.Screen.clearLine();
      H.Screen.print(Lt);
      H.Screen.setCursor(2,5);
      //H.Screen.clearLine();
      H.Screen.print(Rt);*/
      if (Lt == 0 || (q-3 < head && head < q+3)){
        L.spin(directionType::fwd,0,voltageUnits::volt);
        R.spin(directionType::fwd,0,voltageUnits::volt);
      }
      else if (Lt >= Rt){
        turnSped = 12 * (Rt/180);
        if (turnSped<4){
          turnSped = 4;
        }
        L.spin(directionType::fwd,turnSped,voltageUnits::volt);
        R.spin(directionType::rev,turnSped,voltageUnits::volt);
      }
      else if (Lt < Rt){
        turnSped = 12 * (Lt/180);
        if (turnSped<4){
          turnSped = 4;
        }
        L.spin(directionType::rev,turnSped,voltageUnits::volt);
        R.spin(directionType::fwd,turnSped,voltageUnits::volt);
      }
    }
    else {
      L.spin(directionType::fwd,0,voltageUnits::volt);
      R.spin(directionType::fwd,0,voltageUnits::volt);
    }
  }
}
