/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\T0825640Z                                        */
/*    Created:      Wed May 31 2023                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// R                    encoder       G, H            
// L                    encoder       C, D            
// Pist                 digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>

using namespace vex;


motor LF = motor(PORT20, gearSetting::ratio6_1, false);
motor LB = motor(PORT11, gearSetting::ratio6_1, false);
motor RF = motor(PORT3, gearSetting::ratio6_1, true);
motor RB = motor(PORT2, gearSetting::ratio6_1, true);

motor claw = motor(PORT8, gearSetting::ratio18_1, true);

controller H = controller(primary);
double wheelCircumference2 = 3.25 * M_PI; //(( wheel(inch) * 2.54)[convert to cm] * pi)The circumference of the wheel. (cm)
double wheelTrack2 = 15.2; //distance between left and right wheels(inch)


int main() {
  
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  while (true){
    RF.setVelocity(double(H.Axis2.position() - (H.Axis1.position() + H.Axis4.position()/2)), percent);
    LF.setVelocity(double(H.Axis2.position() + (H.Axis1.position() + H.Axis4.position()/2)), percent);
    RB.setVelocity(double(H.Axis2.position() + (H.Axis1.position() - H.Axis4.position()/2)), percent);
    LB.setVelocity(double(H.Axis2.position() - (H.Axis1.position() - H.Axis4.position()/2)), percent);
    RF.spin(forward);
    LF.spin(forward);
    RB.spin(forward);
    LB.spin(forward);
    if (H.ButtonR1.pressing()||H.ButtonB.pressing()){
      claw.spin(directionType::fwd,12,voltageUnits::volt);
    }
    else if (H.ButtonR2.pressing()){
      claw.spin(directionType::rev,12,voltageUnits::volt);
    }
    else{
      claw.stop(brakeType::hold);
    }
    Pist.set(H.ButtonB.pressing());
  }
  /*
  int head;
  int x;
  int y;
  int x2;  int y2;
  int q;
  float Lt;
  float Rt;
  double l;
  double r;
  double turnSped;
  //
  double rf;
  double rb;
  double lf;
  double lb;//
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
    }*/
    /*H.Screen.clearScreen();
    H.Screen.setCursor(1,1);
    //H.Screen.clearLine();
    H.Screen.print(head);*//*
    if (H.Axis3.value()!=0 || H.Axis4.value()!=0){
      x = (H.Axis4.value());// * cos(head*(M_PI/180)) - H.Axis3.value() * sin(head*(M_PI/180)))/10.583;
      y = (H.Axis3.value());// * cos(head*(M_PI/180)) + H.Axis4.value() * sin(head*(M_PI/180)))/10.583;
      rf = y-(x+(H.Axis1.value()/(2*10.583)));
      rb = y+(x-(H.Axis1.value()/(2*10.583)));
      lf = y+(x+(H.Axis1.value()/(2*10.583)));
      lb = y-(x-(H.Axis1.value()/(2*10.583)));
      RF.spin(directionType::fwd,rf,voltageUnits::volt);
      RB.spin(directionType::fwd,rb,voltageUnits::volt);
      LF.spin(directionType::fwd,lf,voltageUnits::volt);
      LB.spin(directionType::fwd,lb,voltageUnits::volt);
    }
    else if (H.Axis1.value()!=0 || H.Axis2.value()!=0){
      //L.spin(directionType::fwd,0,voltageUnits::volt);
      //R.spin(directionType::fwd,0,voltageUnits::volt);
      x2 = H.Axis1.value();
      y2 = H.Axis2.value();
      q = atan2(x2,y2)* (180/M_PI);
      if (q < 0){
        q = 180 + (180-abs(q));
      }*/
      /*
      H.Screen.clearScreen();
      H.Screen.setCursor(1,1);
      //H.Screen.clearLine();
      H.Screen.print(q);
      H.Screen.setCursor(1,5);
      //H.Screen.clearLine();
      H.Screen.print(head);*//*
      Rt = q - head;
      if (Rt < 0){
        Rt = 360 + Rt;
      }
      Lt = head- q;
      if (Lt < 0){
        Lt = 360 + Lt;
      }*/
      /*H.Screen.setCursor(2,1);
      //H.Screen.clearLine();
      H.Screen.print(Lt);
      H.Screen.setCursor(2,5);
      //H.Screen.clearLine();
      H.Screen.print(Rt);*//*
      if (Lt == 0 || (q-10 < head && head < q+10)){
        LF.spin(directionType::fwd,0,voltageUnits::volt);
        LB.spin(directionType::fwd,0,voltageUnits::volt);
        RF.spin(directionType::fwd,0,voltageUnits::volt);
        RB.spin(directionType::fwd,0,voltageUnits::volt);
      }
      else if (Lt >= Rt){
        turnSped = 9 * (Rt/270);
        if (turnSped<4){
          turnSped = 4;
        }
        LF.spin(directionType::fwd,turnSped,voltageUnits::volt);
        LB.spin(directionType::fwd,turnSped,voltageUnits::volt);
        RF.spin(directionType::rev,turnSped,voltageUnits::volt);
        RB.spin(directionType::rev,turnSped,voltageUnits::volt);
      }
      else if (Lt < Rt){
        turnSped = 10 * (Lt/270);
        if (turnSped<4){
          turnSped = 4;
        }
        LF.spin(directionType::rev,turnSped,voltageUnits::volt);
        LB.spin(directionType::rev,turnSped,voltageUnits::volt);
        RF.spin(directionType::fwd,turnSped,voltageUnits::volt);
        RB.spin(directionType::fwd,turnSped,voltageUnits::volt);
      }
    }
    else {
      LF.spin(directionType::fwd,0,voltageUnits::volt);
      LB.spin(directionType::fwd,0,voltageUnits::volt);
      RF.spin(directionType::fwd,0,voltageUnits::volt);
      RB.spin(directionType::fwd,0,voltageUnits::volt);
    }
    if (H.ButtonR1.pressing()){
      claw.spin(directionType::fwd,12,voltageUnits::volt);
    }
    else if (H.ButtonR2.pressing()){
      claw.spin(directionType::rev,12,voltageUnits::volt);
    }
    else{
      claw.stop(brakeType::hold);
    }
  }*/
}
