#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h" 

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// DigitalOutA          digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----

using namespace vex;



// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.

#pragma endregion VEXcode Generated Robot Configuration

// ----------------------------------------------------------------------------
//                                                                            
//    Project:                                               
//    Author:
//    Created:
//    Configuration:        
//                                                                            
// ----------------------------------------------------------------------------

// Include the V5 Library
#include "vex.h"

// Allows for easier use of the VEX Library
using namespace vex;

motor leftfront = motor(PORT17,gearSetting::ratio18_1,false);
motor leftback = motor(PORT13,gearSetting::ratio18_1,true);
motor rightfront = motor(PORT16,gearSetting::ratio18_1,false);
motor rightback = motor(PORT12,gearSetting::ratio18_1,true);

motor topfly = motor(PORT5,gearSetting::ratio6_1,false);
motor botfly = motor(PORT6,gearSetting::ratio6_1,true);
motor intake1 = motor(PORT7,gearSetting::ratio36_1, true);

motor indexer = motor(PORT1,gearSetting::ratio18_1,true);

controller HController = controller(primary);
controller VController = controller(partner);

int maxspeed = 100;
int slwspeed =  .5 * maxspeed;




// Begin project code

void preAutonomous(void) {
  // actions to do when the program starts
  Brain.Screen.clearScreen();
  Brain.Screen.print("pre auton code");
  DigitalOutA.set(true);
  wait(1, seconds);
}

void autonomous(void) {
  Brain.Screen.clearScreen();
  Brain.Screen.print("autonomous code");
  // place automonous code here
  DigitalOutA.set(true);
  
  




}

void userControl(void) {
  Brain.Screen.clearScreen();
  // place driver control in this while loop
  while (true) {



    rightfront.spin(vex::directionType::fwd, (HController.Axis2.value()/8.332),vex::voltageUnits::volt);
    leftfront.spin(vex::directionType::fwd,  (HController.Axis3.value()/8.333),vex::voltageUnits::volt);
    rightback.spin(vex::directionType::fwd,  (HController.Axis2.value()/8.332),vex::voltageUnits::volt);
    leftback.spin(vex::directionType::fwd, (HController.Axis3.value()/8.332),vex::voltageUnits::volt);

    if(VController.ButtonR2.pressing()==1)
    {
      topfly.spin(directionType::fwd, 11, voltageUnits::volt);
      botfly.spin(directionType::fwd, 11, voltageUnits::volt);
    }


    else 
    {
      topfly.spin(directionType::fwd,0,velocityUnits::pct);
      botfly.spin(directionType::fwd,0,velocityUnits::pct);
    }
    if(HController.ButtonR1.pressing()==1)
    {
      indexer.spin(directionType::fwd, 9, voltageUnits::volt);
      
    }



    else 
    {
       indexer.spin(directionType::fwd, 0, voltageUnits::volt);
    }
    if(VController.Axis2.value()<0)//controller prefers variable control
    {
      //intake1.spin(directionType::fwd, 11, voltageUnits::volt);
      //indexer.spin(directionType::fwd, -11, voltageUnits::volt);
      intake1.spin(directionType::rev, VController.Axis2.value(), velocityUnits::pct);
      indexer.spin(directionType::fwd, VController.Axis2.value(), velocityUnits::pct);
      
    }
    else if(VController.Axis2.value()>0)
    {
      //intake1.spin(directionType::fwd, -11, voltageUnits::volt);
      intake1.spin(directionType::rev, VController.Axis2.value(), velocityUnits::pct);
      
    }


    else 
    {
      intake1.spin(directionType::fwd, 0, voltageUnits::volt);
    }

    if(HController.ButtonA.pressing()==1)
    {
      DigitalOutA.set(false);
      

    }
    else{
      DigitalOutA.set(true);
    }

    
    

  }

 


    vex::task::sleep(20);
  }


int main() {
  // create competition instance
  competition Competition;

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  // Run the pre-autonomous function.
  preAutonomous();

  // Prevent main from exiting with an infinite loop.
  while (true) {
 
    wait(1,msec);
  }
}
